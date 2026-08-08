-- ============================================================
--  АВТОНОМНЫЙ БПЛА  |  Create Aeronautics + Avionics + CC:Tweaked
--  Полёт к произвольным координатам без пилота.
--  Позиционирование: navigation_table + лодстоун-компас (якорь),
--  высота: altitude_sensor, стабилизация: gimbal_sensor + гиро-пропеллеры.
--  Сохраните как startup.lua на бортовом компьютере.
-- ============================================================

-- ---------------- КОНФИГ ----------------
local CFG_FILE = "autopilot.cfg"
local cfg = {
  anchor   = { x = 0, y = 64, z = 0 }, -- координаты ЛОДСТОУНА (якоря). Задать: anchor <x> <y> <z>
  s_prime  = nil,   -- знак bearing (автокалибровка)
  h_sign   = nil,   -- ориентация системы координат корпуса (автокалибровка)
  motor_sign = nil, -- знак скорости мотора для тяги вверх (автокалибровка)
  max_tilt = 0.20,  -- гориз. компонент вектора тяги (конус пропеллера ~12° => max 0.21)
  v_max    = 9,     -- целевая крейсерская скорость, блоков/с
  arrive_r = 2.5,   -- радиус прибытия, блоки
  clearance = 8,    -- запас высоты на маршруте
  alt = { kp = 22, ki = 3.5, kd = 30 },  -- PID высоты -> скорость мотора
  hor = { kp = 0.30, kd = 0.055 },       -- PD горизонтали -> наклон тяги
}
local function saveCfg()
  local f = fs.open(CFG_FILE, "w"); f.write(textutils.serialize(cfg)); f.close()
end
if fs.exists(CFG_FILE) then
  local f = fs.open(CFG_FILE, "r")
  local ok, t = pcall(textutils.unserialize, f.readAll()); f.close()
  if ok and type(t) == "table" then for k, v in pairs(t) do cfg[k] = v end end
end

-- ---------------- ПЕРИФЕРИЯ ----------------
local props = { peripheral.find("gyroscopic_propeller_bearing") }
local motor = peripheral.find("Create_CreativeMotor")
local nav   = peripheral.find("navigation_table")
local alt   = peripheral.find("altitude_sensor")
local gyro  = peripheral.find("gimbal_sensor")
local pa    = peripheral.find("physics_assembler")

local function require_periph(p, n, want)
  if not p or (want and #p ~= want) then
    error(("Периферия '%s' не найдена%s. Проверь кабели/модемы."):format(
      n, want and (" (нужно " .. want .. ", есть " .. (p and #p or 0) .. ")") or ""), 0)
  end
end
require_periph(props[1] and props, "gyroscopic_propeller_bearing", nil)
if #props ~= 4 then error("Найдено пропеллеров: " .. #props .. " из 4. Проверь сеть.", 0) end
require_periph(motor, "Create_CreativeMotor")
require_periph(nav, "navigation_table")
require_periph(alt, "altitude_sensor")
require_periph(gyro, "gimbal_sensor")

-- ---------------- СОСТОЯНИЕ ----------------
local state = "IDLE"       -- IDLE|ARM|CAL|TAKEOFF|NAV|HOVER|LAND
local goal = nil           -- {x,y,z, land=bool}
local est = { x=0, z=0, y=0, vx=0, vz=0, vy=0, ok=false }
local ui_msg = "введите команду (help)"
local alt_target, transit_alt = nil, nil
local alt_I = 0
local DT = 0.15            -- период контура, с
local last_h, last_x, last_z = nil, nil, nil

-- ---------------- УТИЛИТЫ ----------------
local function clamp(v, lo, hi) return math.max(lo, math.min(hi, v)) end
local function wrap180(a) return (a + 180) % 360 - 180 end

local function allProps(fn, ...)
  for _, p in ipairs(props) do pcall(p[fn], ...) end
end
local function setTilt(bx, bz)              -- вектор тяги в системе корпуса
  local m = math.sqrt(bx*bx + bz*bz)
  if m > cfg.max_tilt then bx, bz = bx*cfg.max_tilt/m, bz*cfg.max_tilt/m end
  for _, p in ipairs(props) do pcall(p.setManualTarget, { bx, 1, bz }) end
end
local function tiltNeutral() allProps("clearManualTarget") end -- гиро сам держит "вверх"
local function setMotor(v)
  pcall(motor.setGeneratedSpeed, clamp(math.floor(v + 0.5), -256, 256))
end

-- ---------------- НАВИГАЦИЯ ----------------
-- Мировая азимут-конвенция: az(dir)=atan2(dx,dz); dir(az)=(sin az, cos az). Градусы.
local function readSensors()
  local h = alt.getHeight()
  est.vy = last_h and (h - last_h) / DT or 0
  last_h = h
  est.y = h

  if not nav.hasTarget() then est.ok = false; return end
  local d3   = nav.getDistanceToTarget()
  local dyA  = nav.getVerticalOffsetToTarget()          -- anchor.y - self.y
  local brg  = nav.getBearing()                         -- цель отн. носа, [-180,180]
  local hdg  = nav.getHeading()                         -- курс носа в мире
  local dh   = math.sqrt(math.max(d3*d3 - dyA*dyA, 0))
  local sp, hs = cfg.s_prime or 1, cfg.h_sign or 1
  -- мировой азимут на якорь: hdg + h*S'*bearing
  local azA = hdg + hs * sp * brg
  local sx = cfg.anchor.x - dh * math.sin(math.rad(azA))
  local sz = cfg.anchor.z - dh * math.cos(math.rad(azA))
  local vx = last_x and (sx - last_x) / DT or 0
  local vz = last_z and (sz - last_z) / DT or 0
  est.vx = est.vx * 0.7 + vx * 0.3                      -- сглаживание
  est.vz = est.vz * 0.7 + vz * 0.3
  last_x, last_z = sx, sz
  est.x, est.z = sx, sz
  est.hdg, est.brg, est.dh = hdg, brg, dh
  est.ok = true
end

-- направление на мировую точку в системе корпуса
local function bodyDirTo(wx, wz)
  local az = math.deg(math.atan2(wx - est.x, wz - est.z))    -- мировой азимут на цель
  local beta = (cfg.h_sign or 1) * wrap180(az - est.hdg)     -- в корпусе
  return math.sin(math.rad(beta)), math.cos(math.rad(beta))
end

-- ---------------- КОНТУР ВЫСОТЫ ----------------
local function altitudeLoop()
  if not alt_target then return end
  local e = alt_target - est.y
  alt_I = clamp(alt_I + e * cfg.alt.ki * DT, -256, 256)      -- I-часть выучит газ висения
  local out = alt_I + cfg.alt.kp * e - cfg.alt.kd * est.vy
  setMotor((cfg.motor_sign or 1) * clamp(out, 0, 256))
end

-- ---------------- КАЛИБРОВКИ ----------------
local function calibrateThrust()
  ui_msg = "CAL: направление тяги..."
  alt_target = nil
  for _, sgn in ipairs({ 1, -1 }) do
    local h0 = alt.getHeight()
    setMotor(sgn * 160); sleep(2.5)
    local dh = alt.getHeight() - h0
    setMotor(sgn * 60)                       -- не глушить: мягко удерживаем
    if dh > 0.5 then cfg.motor_sign = sgn; saveCfg(); return true end
    setMotor(0); sleep(1.5)
  end
  return false
end

local function calibrateBearing()
  ui_msg = "CAL: знак пеленга..."
  for _, sp in ipairs({ 1, -1 }) do
    cfg.s_prime = sp
    local d0 = nav.getDistanceToTarget()
    local t0 = os.clock()
    while os.clock() - t0 < 4 do             -- наклон строго на якорь
      readSensors(); altitudeLoop()
      local b = math.rad(sp * nav.getBearing())
      setTilt(math.sin(b) * 0.10, math.cos(b) * 0.10)
      sleep(DT)
    end
    tiltNeutral()
    if nav.getDistanceToTarget() < d0 - 1.0 then saveCfg(); return true end
    sleep(2)                                  -- погасить скорость
  end
  return false
end

local function calibrateFrame()
  ui_msg = "CAL: ориентация корпуса..."
  readSensors()
  -- гипотезы h=±1 дают зеркальные позиции; двигаемся строго "вперёд" (beta=0),
  -- у верной гипотезы азимут смещения совпадает с курсом
  local snap = {}
  for _, hh in ipairs({ 1, -1 }) do
    cfg.h_sign = hh; readSensors()
    snap[hh] = { x = est.x, z = est.z }
  end
  local t0 = os.clock()
  while os.clock() - t0 < 4 do
    readSensors(); altitudeLoop()
    setTilt(0, 0.10)                          -- вперёд по корпусу
    sleep(DT)
  end
  tiltNeutral()
  local best, bestErr = 1, 1e9
  for _, hh in ipairs({ 1, -1 }) do
    cfg.h_sign = hh; readSensors()
    local dx, dz = est.x - snap[hh].x, est.z - snap[hh].z
    if dx*dx + dz*dz > 0.5 then
      local azm = math.deg(math.atan2(dx, dz))
      local err = math.abs(wrap180(azm - est.hdg))
      if err < bestErr then best, bestErr = hh, err end
    end
  end
  cfg.h_sign = best; saveCfg()
  sleep(1.5)
  return bestErr < 60
end

-- ---------------- ГЛАВНЫЙ КОНТУР ----------------
local function safetyOK()
  local a = gyro.getAngles()
  local tilt = math.max(math.abs(a[1] or 0), math.abs(a[2] or 0))
  if tilt > 40 then
    tiltNeutral(); ui_msg = ("КРЕН %.0f°! стабилизация..."):format(tilt)
    return false
  end
  return true
end

local function controlLoop()
  while true do
    readSensors()

    if state == "ARM" then
      allProps("assemble")
      ui_msg = "роторы собраны. Кликни Physics Assembler для сборки корпуса!"
      if not pa or pa.isAssembled() then state = "CAL" end

    elseif state == "CAL" then
      if not est.ok then
        ui_msg = "нет цели в нав-столе! Положи лодстоун-компас."
      else
        if not cfg.motor_sign and not calibrateThrust() then
          ui_msg = "тяга не поднимает аппарат — проверь роторы"; state = "IDLE"
        else
          alt_target = est.y + 6
          if not cfg.s_prime  then calibrateBearing() end
          if not cfg.h_sign   then calibrateFrame()   end
          ui_msg = "калибровка готова"; state = goal and "TAKEOFF" or "HOVER"
        end
      end

    elseif state == "TAKEOFF" then
      transit_alt = math.max(est.y, goal.y) + cfg.clearance
      alt_target = transit_alt
      if math.abs(est.y - transit_alt) < 2 then state = "NAV" end
      altitudeLoop()

    elseif state == "NAV" then
      altitudeLoop()
      if not est.ok then
        tiltNeutral(); ui_msg = "потерян якорь! верни компас в нав-стол"
      elseif safetyOK() then
        local dx, dz = goal.x - est.x, goal.z - est.z
        local dist = math.sqrt(dx*dx + dz*dz)
        if dist < cfg.arrive_r then
          tiltNeutral(); alt_target = goal.y
          state = goal.land and "LAND" or "HOVER"
          ui_msg = "прибыли: " .. ("%.1f, %.1f"):format(est.x, est.z)
        else
          local bx, bz = bodyDirTo(goal.x, goal.z)
          local v = math.sqrt(est.vx^2 + est.vz^2)
          local v_des = clamp(dist * 0.5, 0, cfg.v_max)
          local mag = clamp(cfg.hor.kp * (v_des - v) * 0.1 + dist * 0.01, 0, cfg.max_tilt)
          -- демпфер: компонент скорости в корпусе
          local dvx, dvz = bodyDirTo(est.x + est.vx, est.z + est.vz)
          local vb = v
          setTilt(bx * mag - dvx * vb * cfg.hor.kd, bz * mag - dvz * vb * cfg.hor.kd)
          ui_msg = ("NAV: %.0fм v=%.1f h=%.0f"):format(dist, v, est.y)
        end
      end

    elseif state == "HOVER" then
      altitudeLoop()
      if safetyOK() then
        -- мягкое удержание точки
        if goal then
          local dx, dz = goal.x - est.x, goal.z - est.z
          local d = math.sqrt(dx*dx + dz*dz)
          if d > 1 then
            local bx, bz = bodyDirTo(goal.x, goal.z)
            setTilt(bx * clamp(d * 0.02, 0, 0.08), bz * clamp(d * 0.02, 0, 0.08))
          else tiltNeutral() end
        end
      end

    elseif state == "LAND" then
      safetyOK()
      alt_target = (alt_target or est.y) - 1.6 * DT       -- ~1.6 блока/с вниз
      altitudeLoop()
      if est.vy > -0.08 and (alt_target < est.y - 3) then  -- команда вниз есть, а не снижаемся => сели
        setMotor(0); tiltNeutral(); alt_I = 0; alt_target = nil
        state = "IDLE"; ui_msg = "посадка выполнена"
      end
    end

    sleep(DT)
  end
end

-- ---------------- ИНТЕРФЕЙС ----------------
local HELP = [[
Команды:
 anchor <x> <y> <z>  координаты лодстоуна-якоря
 goto <x> <y> <z>    лететь к точке (виснуть по прибытии)
 fly <x> <y> <z>     лететь и сесть в точке
 home                лететь к якорю
 hover               зависнуть на месте
 land                посадка здесь
 stop                мотор 0, всё сбросить
 recal               сбросить калибровки
 st                  статус]]

local function startMission(x, y, z, land_flag)
  goal = { x = x, y = y, z = z, land = land_flag }
  alt_I = 0
  if state == "IDLE" then state = "ARM"
  elseif state == "HOVER" or state == "NAV" or state == "LAND" then state = "TAKEOFF" end
end

local function uiLoop()
  term.clear()
  while true do
    term.setCursorPos(1, 1); term.clearLine()
    write(("[%s] %s"):format(state, ui_msg))
    term.setCursorPos(1, 2); term.clearLine()
    if est.ok then
      write(("поз: %.0f %.0f %.0f  якорь:%.0f,%.0f,%.0f"):format(
        est.x, est.y, est.z, cfg.anchor.x, cfg.anchor.y, cfg.anchor.z))
    else
      write("нет позиции: проверь компас в нав-столе")
    end
    term.setCursorPos(1, 4); term.clearLine(); write("> ")
    local line = read()
    local a = {}
    for w in line:gmatch("%S+") do a[#a+1] = w end
    local c = (a[1] or ""):lower()
    if c == "anchor" and #a == 4 then
      cfg.anchor = { x = tonumber(a[2]), y = tonumber(a[3]), z = tonumber(a[4]) }
      saveCfg(); ui_msg = "якорь сохранён"
    elseif (c == "goto" or c == "fly") and #a == 4 then
      startMission(tonumber(a[2]), tonumber(a[3]), tonumber(a[4]), c == "fly")
    elseif c == "home" then
      startMission(cfg.anchor.x, cfg.anchor.y + cfg.clearance, cfg.anchor.z, false)
    elseif c == "hover" then
      goal = nil; tiltNeutral(); alt_target = est.y; state = "HOVER"
    elseif c == "land" then
      goal = nil; tiltNeutral(); state = "LAND"
    elseif c == "stop" then
      setMotor(0); tiltNeutral(); alt_target = nil; goal = nil; state = "IDLE"
      ui_msg = "остановлено"
    elseif c == "recal" then
      cfg.s_prime, cfg.h_sign, cfg.motor_sign = nil, nil, nil; saveCfg()
      ui_msg = "калибровки сброшены"
    elseif c == "st" then
      ui_msg = ("props=%d asm=%s v=(%.1f %.1f %.1f)"):format(
        #props, tostring(pa and pa.isAssembled()), est.vx, est.vy, est.vz)
    elseif c == "help" or c == "" then
      term.setCursorPos(1, 6); print(HELP)
    else
      ui_msg = "не понял. help — список команд"
    end
  end
end

-- ---------------- СТАРТ ----------------
allProps("setThrustHandedness", "right_handed")  -- одинаковый шаг лопастей на всех
print("Автопилот запущен. help — команды.")
parallel.waitForAny(controlLoop, uiLoop)


-- wget https://raw.githubusercontent.com/ISSSSA/minecraft-data/main/probe.lua probe