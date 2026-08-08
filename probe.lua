-- ============================================================
--  AUTONOMOUS DRONE v3 - flight controller edition
--  Create Aeronautics + Avionics + CC:Tweaked
--
--  New in v3 (per-rotor runtime control):
--   * each propeller has its own Rotation Speed Controller (RSC) = "ESC"
--   * attitude PID on the gimbal sensor (gyroscope): trims out any
--     payload imbalance ("perewes") with differential rotor RPM
--   * automatic rotor mapping: pulses each RSC and reads the gyro
--     response to learn which rotor sits in which corner
--   * blade pitch auto-normalization on the ground via getThrust()
--  Save as startup.lua on the onboard computer.
-- ============================================================

-- ---------------- CONFIG ----------------
-- Change at runtime with:  set <key> <value>  (e.g. "set alt_ceiling 60",
-- "set att.kp 400"). Values persist in autopilot.cfg.
local CFG_FILE = "autopilot.cfg"
local cfg = {
  anchor   = { x = 0, y = 64, z = 0 }, -- LODESTONE coords. Set with: anchor <x> <y> <z>
  alt_ceiling = 30, -- max height above launch point, blocks
  s_prime  = nil,   -- bearing sign                 (auto-calibrated)
  h_sign   = nil,   -- body frame handedness        (auto-calibrated)
  rsc_sign = nil,   -- rotor spin sign for upthrust (auto-calibrated)
  att_map  = nil,   -- rotor -> tilt response map   (auto-calibrated)
  ground_y = nil,   -- launch altitude, captured on arming
  max_tilt = 0.20,  -- horizontal thrust component (prop cone ~12 deg => max 0.21)
  tilt_slew = 0.05, -- max tilt change per control tick
  v_max    = 9,     -- cruise speed cap, blocks/s
  a_brake  = 2.0,   -- braking decel for the approach curve, blocks/s^2
  arrive_r = 2.5,   -- arrival radius, blocks
  clearance = 8,    -- transit altitude margin
  land_rate = 1.6,  -- landing descent speed, blocks/s
  pulse    = 50,    -- rotor-mapping pulse, RPM
  alt = { kp = 22, ki = 3.5, kd = 30 },   -- altitude PID -> collective RPM
  hor = { kv = 0.045, kd = 0.012 },       -- velocity PD -> thrust tilt
  att = { kp = 340, ki = 30, kd = 110 },  -- attitude PID (gyro) -> differential RPM
}
local function saveCfg()
  local f = fs.open(CFG_FILE, "w"); f.write(textutils.serialize(cfg)); f.close()
end
if fs.exists(CFG_FILE) then
  local f = fs.open(CFG_FILE, "r")
  local ok, t = pcall(textutils.unserialize, f.readAll()); f.close()
  if ok and type(t) == "table" then for k, v in pairs(t) do cfg[k] = v end end
end

-- ---------------- PERIPHERALS ----------------
local props = { peripheral.find("gyroscopic_propeller_bearing") }
local rscs  = { peripheral.find("Create_RotationSpeedController") }
local motor = peripheral.find("Create_CreativeMotor")
local nav   = peripheral.find("navigation_table")
local alt   = peripheral.find("altitude_sensor")
local gyro  = peripheral.find("gimbal_sensor")
local pa    = peripheral.find("physics_assembler")

local function need(p, n)
  if not p then error(("Peripheral '%s' not found. Check cables/modems."):format(n), 0) end
end
if #props ~= 4 then error("Found " .. #props .. "/4 propellers. Check the network.", 0) end
if #rscs  ~= 4 then error("Found " .. #rscs .. "/4 speed controllers. Check the network.", 0) end
need(motor, "Create_CreativeMotor"); need(nav, "navigation_table")
need(alt, "altitude_sensor"); need(gyro, "gimbal_sensor")

-- ---------------- STATE ----------------
local state = "IDLE"       -- IDLE|ARM|CAL|TAKEOFF|NAV|HOVER|LAND
local goal = nil           -- {x,y,z, land=bool}
local wps = {}             -- waypoint queue
local est = { x=0, z=0, y=0, vx=0, vz=0, vy=0, ok=false }
local ui_msg = "type a command (help)"
local alt_target, alt_I = nil, 0
local att_I = { x = 0, z = 0 }
local t_prev = { x = 0, z = 0 }
local pulse_rsc, pulse_amt = nil, 0    -- rotor-mapping injection
local cur_bx, cur_bz = 0, 0
local DT = 0.15
local last_h, last_x, last_z = nil, nil, nil

-- ---------------- UTILS ----------------
local function clamp(v, lo, hi) return math.max(lo, math.min(hi, v)) end
local function wrap180(a) return (a + 180) % 360 - 180 end
local function allProps(fn, ...)
  for _, p in ipairs(props) do pcall(p[fn], ...) end
end
local function ceilingY()
  return (cfg.ground_y or est.y) + cfg.alt_ceiling
end

-- thrust-vectoring (translation): same tilt on all four rotors
local function rawTilt(bx, bz)
  local m = math.sqrt(bx*bx + bz*bz)
  if m > cfg.max_tilt then bx, bz = bx*cfg.max_tilt/m, bz*cfg.max_tilt/m end
  for _, p in ipairs(props) do pcall(p.setManualTarget, { bx, 1, bz }) end
  cur_bx, cur_bz = bx, bz
end
local function setTilt(bx, bz)
  local dx, dz = bx - cur_bx, bz - cur_bz
  local d = math.sqrt(dx*dx + dz*dz)
  if d > cfg.tilt_slew then
    bx = cur_bx + dx / d * cfg.tilt_slew
    bz = cur_bz + dz / d * cfg.tilt_slew
  end
  rawTilt(bx, bz)
end
local function tiltNeutral()
  allProps("clearManualTarget"); cur_bx, cur_bz = 0, 0
end

-- per-rotor RPM: collective (altitude) + differential (attitude) + mapping pulse
local function applyRotors(base, cx, cz)
  for _, r in ipairs(rscs) do
    local nm = peripheral.getName(r)
    local d = cfg.att_map and cfg.att_map[nm]
    local corr = d and (cx * d.x + cz * d.z) or 0
    if r == pulse_rsc then corr = corr + pulse_amt end
    local rpm = clamp(base + corr, 0, 256)
    pcall(r.setTargetSpeed, math.floor((cfg.rsc_sign or 1) * rpm + 0.5))
  end
end
local function rotorsOff()
  for _, r in ipairs(rscs) do pcall(r.setTargetSpeed, 0) end
end

-- ---------------- SENSORS / NAVIGATION ----------------
-- World azimuth convention: az(dir)=atan2(dx,dz); dir(az)=(sin az, cos az).
local function readSensors()
  local h = alt.getHeight()
  est.vy = last_h and (h - last_h) / DT or 0
  last_h = h
  est.y = h

  if not nav.hasTarget() then est.ok = false; return end
  local d3   = nav.getDistanceToTarget()
  local dyA  = nav.getVerticalOffsetToTarget()
  local brg  = nav.getBearing()
  local hdg  = nav.getHeading()
  local dh   = math.sqrt(math.max(d3*d3 - dyA*dyA, 0))
  local sp, hs = cfg.s_prime or 1, cfg.h_sign or 1
  local azA = hdg + hs * sp * brg
  local sx = cfg.anchor.x - dh * math.sin(math.rad(azA))
  local sz = cfg.anchor.z - dh * math.cos(math.rad(azA))
  local vx = last_x and (sx - last_x) / DT or 0
  local vz = last_z and (sz - last_z) / DT or 0
  est.vx = est.vx * 0.7 + vx * 0.3
  est.vz = est.vz * 0.7 + vz * 0.3
  last_x, last_z = sx, sz
  est.x, est.z = sx, sz
  est.hdg, est.brg, est.dh = hdg, brg, dh
  est.ok = true
end

local function worldToBody(wx, wz)
  local hd = math.rad(est.hdg or 0)
  local along = wx * math.sin(hd) + wz * math.cos(hd)
  local lat   = wx * math.cos(hd) - wz * math.sin(hd)
  return (cfg.h_sign or 1) * lat, along
end

-- tilt proxy from the GYROSCOPE: horizontal gravity components in body frame.
-- (0,0) = perfectly level. Same basis is used for calibration and control,
-- so no sign conventions can ever mismatch.
local function gravityTilt()
  local g = gyro.getGravity()
  local gx, gy, gz = g[1] or 0, g[2] or 0, g[3] or 0
  local m = math.sqrt(gx*gx + gy*gy + gz*gz)
  if m < 0.05 then return 0, 0, false end
  return gx / m, gz / m, true
end

-- ---------------- CONTROL LOOPS ----------------
local function altitudeLoop()               -- -> collective RPM
  if not alt_target then return 0 end
  alt_target = math.min(alt_target, ceilingY())
  local e = alt_target - est.y
  local out = alt_I + cfg.alt.kp * e - cfg.alt.kd * est.vy
  if out > 0 and out < 256 then             -- anti-windup
    alt_I = clamp(alt_I + e * cfg.alt.ki * DT, 0, 256)
  end
  return clamp(out, 0, 256)
end

-- attitude PID: fights static imbalance (payload offset) and disturbances
-- with differential rotor thrust. Integral term = automatic trim.
local function attitudeLoop(active)
  if not active then t_prev.x, t_prev.z = 0, 0; return 0, 0 end
  local tx, tz, okg = gravityTilt()
  if not okg then return 0, 0 end
  local dtx = (tx - t_prev.x) / DT
  local dtz = (tz - t_prev.z) / DT
  t_prev.x, t_prev.z = tx, tz
  att_I.x = clamp(att_I.x + tx * cfg.att.ki * DT, -70, 70)
  att_I.z = clamp(att_I.z + tz * cfg.att.ki * DT, -70, 70)
  local cx = -(cfg.att.kp * tx + cfg.att.kd * dtx) - att_I.x
  local cz = -(cfg.att.kp * tz + cfg.att.kd * dtz) - att_I.z
  return cx, cz
end

local airborne_states = { TAKEOFF = true, NAV = true, HOVER = true, LAND = true }
local function updateRotors()
  local base = altitudeLoop()
  local cx, cz = attitudeLoop(airborne_states[state] and cfg.att_map ~= nil)
  applyRotors(base, cx, cz)
end

local prev_evx, prev_evz = 0, 0
local function flyToward(tx, tz, speed_cap)
  local dx, dz = tx - est.x, tz - est.z
  local dist = math.sqrt(dx*dx + dz*dz)
  if dist < 0.05 then setTilt(0, 0); return dist end
  local v_des = math.min(speed_cap or cfg.v_max,
                         math.sqrt(2 * cfg.a_brake * math.max(dist - cfg.arrive_r * 0.5, 0)))
  local vdx, vdz = dx / dist * v_des, dz / dist * v_des
  local evx, evz = vdx - est.vx, vdz - est.vz
  local ax = cfg.hor.kv * evx + cfg.hor.kd * (evx - prev_evx) / DT
  local az = cfg.hor.kv * evz + cfg.hor.kd * (evz - prev_evz) / DT
  prev_evx, prev_evz = evx, evz
  local bx, bz = worldToBody(ax, az)
  setTilt(bx, bz)
  return dist
end

-- ---------------- CALIBRATION ----------------
-- 1) ground: normalize blade pitch so every rotor thrusts UP
local function calibrateBlades()
  ui_msg = "CAL: blade pitch..."
  applyRotors(90, 0, 0); sleep(2.0)
  local flipped, bad = 0, 0
  for _, p in ipairs(props) do
    local okc, th = pcall(p.getThrust)
    if okc and th and th < -0.01 then
      local h = p.getThrustHandedness()
      p.setThrustHandedness(h == "right_handed" and "left_handed" or "right_handed")
      flipped = flipped + 1
    elseif not okc or not th or math.abs(th) < 0.01 then
      bad = bad + 1
    end
  end
  sleep(0.8)
  return bad == 0, flipped
end

-- 2) ground->air: does collective actually lift? if not, flip global spin
local function calibrateLift()
  ui_msg = "CAL: lift test..."
  for _, sgn in ipairs({ cfg.rsc_sign or 1, -(cfg.rsc_sign or 1) }) do
    cfg.rsc_sign = sgn
    calibrateBlades()                       -- re-check pitch for this spin dir
    local h0 = alt.getHeight()
    applyRotors(180, 0, 0); sleep(2.5)
    if alt.getHeight() - h0 > 0.5 then
      applyRotors(70, 0, 0); saveCfg(); return true
    end
    rotorsOff(); sleep(1.5)
  end
  return false
end

-- 3) airborne: learn which RSC drives which corner by pulsing each rotor
--    and reading the tilt response on the GYROSCOPE
local function sampleTilt(dur)
  local sx, sz, n = 0, 0, 0
  local t0 = os.clock()
  while os.clock() - t0 < dur do
    readSensors(); updateRotors()
    local tx, tz = gravityTilt()
    sx, sz, n = sx + tx, sz + tz, n + 1
    sleep(DT)
  end
  return sx / n, sz / n
end

local function calibrateRotorMap()
  ui_msg = "CAL: rotor mapping (gyro)..."
  local map = {}
  cfg.att_map = nil                          -- raw pulses, no attitude loop
  for i, r in ipairs(rscs) do
    local b0x, b0z = sampleTilt(0.8)         -- baseline
    pulse_rsc, pulse_amt = r, cfg.pulse
    local t0 = os.clock()                    -- settle into the pulse
    while os.clock() - t0 < 0.7 do readSensors(); updateRotors(); sleep(DT) end
    local px, pz = sampleTilt(0.9)           -- response
    pulse_rsc, pulse_amt = nil, 0
    local dx, dz = px - b0x, pz - b0z
    local m = math.sqrt(dx*dx + dz*dz)
    ui_msg = ("CAL: rotor %d/4 response %.3f"):format(i, m)
    if m < 0.004 then return false end       -- no measurable response
    map[peripheral.getName(r)] = { x = dx / m, z = dz / m }
    local t1 = os.clock()                    -- let it re-level
    while os.clock() - t1 < 1.2 do readSensors(); updateRotors(); sleep(DT) end
  end
  cfg.att_map = map; saveCfg()
  return true
end

-- 4) airborne: bearing sign (tilt straight at the anchor, expect closure)
local function calibrateBearing()
  ui_msg = "CAL: bearing sign..."
  for _, sp in ipairs({ 1, -1 }) do
    cfg.s_prime = sp
    local d0 = nav.getDistanceToTarget()
    local t0 = os.clock()
    while os.clock() - t0 < 4 do
      readSensors(); updateRotors()
      local b = math.rad(sp * nav.getBearing())
      rawTilt(math.sin(b) * 0.10, math.cos(b) * 0.10)
      sleep(DT)
    end
    tiltNeutral()
    if nav.getDistanceToTarget() < d0 - 1.0 then saveCfg(); return true end
    local t1 = os.clock()
    while os.clock() - t1 < 2 do readSensors(); updateRotors(); sleep(DT) end
  end
  return false
end

-- 5) airborne: body frame handedness (move forward, compare azimuths)
local function calibrateFrame()
  ui_msg = "CAL: body frame..."
  readSensors()
  local snap = {}
  for _, hh in ipairs({ 1, -1 }) do
    cfg.h_sign = hh; readSensors()
    snap[hh] = { x = est.x, z = est.z }
  end
  local t0 = os.clock()
  while os.clock() - t0 < 4 do
    readSensors(); updateRotors()
    rawTilt(0, 0.10)
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
  local t1 = os.clock()
  while os.clock() - t1 < 1.5 do readSensors(); updateRotors(); sleep(DT) end
  return bestErr < 60
end

-- ---------------- MAIN LOOP ----------------
local function safetyOK()
  local tx, tz = gravityTilt()
  local tilt = math.deg(math.asin(clamp(math.sqrt(tx*tx + tz*tz), 0, 1)))
  if tilt > 40 then
    tiltNeutral(); ui_msg = ("TILT %.0f deg! recovering..."):format(tilt)
    return false
  end
  return true
end

local function nextGoal()
  if #wps > 0 then
    goal = table.remove(wps, 1)
    state = "TAKEOFF"
    ui_msg = ("next waypoint: %.0f %.0f %.0f (%d left)"):format(goal.x, goal.y, goal.z, #wps)
    return true
  end
  return false
end

local function controlLoop()
  while true do
    readSensors()

    if state == "ARM" then
      cfg.ground_y = est.y; saveCfg()
      pcall(motor.setGeneratedSpeed, 256)    -- constant source; RSCs do the mixing
      allProps("assemble")
      ui_msg = "rotors assembled. Click the Physics Assembler!"
      if not pa or pa.isAssembled() then state = "CAL" end

    elseif state == "CAL" then
      if not est.ok then
        ui_msg = "no target in nav table! Insert a lodestone compass."
      elseif not cfg.rsc_sign and not calibrateLift() then
        ui_msg = "cannot lift - check rotors"; rotorsOff(); state = "IDLE"
      else
        alt_target = math.min(est.y + 6, ceilingY())
        att_I.x, att_I.z = 0, 0
        if not cfg.att_map  then calibrateRotorMap() end
        if not cfg.s_prime  then calibrateBearing()  end
        if not cfg.h_sign   then calibrateFrame()    end
        ui_msg = cfg.att_map and "calibration done (attitude trim ON)"
                              or "calibration done (no rotor map!)"
        state = goal and "TAKEOFF" or "HOVER"
      end
      updateRotors()

    elseif state == "TAKEOFF" then
      local transit = math.max(est.y, goal.y) + cfg.clearance
      alt_target = math.min(transit, ceilingY())
      if math.abs(est.y - alt_target) < 2 then state = "NAV" end
      updateRotors()

    elseif state == "NAV" then
      updateRotors()
      if not est.ok then
        tiltNeutral(); ui_msg = "anchor lost! Put the compass back"
      elseif safetyOK() then
        local dist = flyToward(goal.x, goal.z)
        if dist < cfg.arrive_r then
          tiltNeutral()
          alt_target = math.min(goal.y, ceilingY())
          if goal.land then state = "LAND"
          elseif not nextGoal() then
            state = "HOVER"
            ui_msg = ("arrived: %.1f, %.1f"):format(est.x, est.z)
          end
        else
          local v = math.sqrt(est.vx^2 + est.vz^2)
          ui_msg = ("NAV: %.0fm v=%.1f h=%.0f/%d"):format(dist, v, est.y, ceilingY())
        end
      end

    elseif state == "HOVER" then
      updateRotors()
      if est.ok and safetyOK() and goal then
        flyToward(goal.x, goal.z, 2.0)
      end

    elseif state == "LAND" then
      safetyOK()
      alt_target = (alt_target or est.y) - cfg.land_rate * DT
      updateRotors()
      if est.vy > -0.08 and (alt_target < est.y - 3) then
        rotorsOff(); pcall(motor.setGeneratedSpeed, 0)
        tiltNeutral(); alt_I = 0; alt_target = nil
        att_I.x, att_I.z = 0, 0
        if nextGoal() then pcall(motor.setGeneratedSpeed, 256)
        else state = "IDLE"; ui_msg = "landed" end
      end
    end

    sleep(DT)
  end
end

-- ---------------- UI ----------------
local HELP = [[
Commands:
 anchor <x> <y> <z>  lodestone (anchor) coords
 goto <x> <y> <z>    fly to point, hover there
 fly <x> <y> <z>     fly to point and land
 wp <x> <y> <z>      queue waypoint (wp clear / wp list)
 go [land]           fly the waypoint queue
 home / hover / land / stop
 set <key> <value>   change config (set alt_ceiling 60, set att.kp 400)
 cfg                 show config    recal: reset calibration
 trim                show attitude trim (imbalance compensation)
 st                  telemetry]]

local function startMission(x, y, z, land_flag)
  goal = { x = x, y = math.min(y, ceilingY()), z = z, land = land_flag }
  alt_I = 0
  if state == "IDLE" then state = "ARM"
  elseif state ~= "ARM" and state ~= "CAL" then state = "TAKEOFF" end
end

local function setKey(path, val)
  local node, last = cfg, nil
  for part in path:gmatch("[^.]+") do
    if last then
      if type(node[last]) ~= "table" then return false end
      node = node[last]
    end
    last = part
  end
  if node[last] == nil or type(node[last]) == "table" then return false end
  local n = tonumber(val)
  if n == nil then return false end
  node[last] = n; saveCfg(); return true
end

local function uiLoop()
  term.clear()
  while true do
    term.setCursorPos(1, 1); term.clearLine()
    write(("[%s] %s"):format(state, ui_msg))
    term.setCursorPos(1, 2); term.clearLine()
    if est.ok then
      write(("pos %.0f %.0f %.0f  ceil %d  wps %d"):format(
        est.x, est.y, est.z, ceilingY(), #wps))
    else
      write("no position: check compass in nav table")
    end
    term.setCursorPos(1, 4); term.clearLine(); write("> ")
    local line = read()
    local a = {}
    for w in line:gmatch("%S+") do a[#a+1] = w end
    local c = (a[1] or ""):lower()
    if c == "anchor" and #a == 4 then
      cfg.anchor = { x = tonumber(a[2]), y = tonumber(a[3]), z = tonumber(a[4]) }
      saveCfg(); ui_msg = "anchor saved"
    elseif (c == "goto" or c == "fly") and #a == 4 then
      wps = {}
      startMission(tonumber(a[2]), tonumber(a[3]), tonumber(a[4]), c == "fly")
    elseif c == "wp" and a[2] == "clear" then
      wps = {}; ui_msg = "waypoints cleared"
    elseif c == "wp" and a[2] == "list" then
      ui_msg = #wps .. " waypoint(s) queued"
      term.setCursorPos(1, 6)
      for i, w in ipairs(wps) do print(("%d: %.0f %.0f %.0f"):format(i, w.x, w.y, w.z)) end
    elseif c == "wp" and #a == 4 then
      wps[#wps+1] = { x = tonumber(a[2]), y = math.min(tonumber(a[3]), ceilingY()),
                      z = tonumber(a[4]) }
      ui_msg = "waypoint #" .. #wps .. " added"
    elseif c == "go" then
      if #wps == 0 then ui_msg = "queue is empty (wp x y z)"
      else
        if a[2] == "land" then wps[#wps].land = true end
        local first = table.remove(wps, 1)
        startMission(first.x, first.y, first.z, first.land)
      end
    elseif c == "home" then
      wps = {}
      startMission(cfg.anchor.x, cfg.anchor.y + cfg.clearance, cfg.anchor.z, false)
    elseif c == "hover" then
      goal = nil; wps = {}; tiltNeutral(); alt_target = est.y; state = "HOVER"
    elseif c == "land" then
      goal = nil; wps = {}; tiltNeutral(); state = "LAND"
    elseif c == "stop" then
      rotorsOff(); pcall(motor.setGeneratedSpeed, 0)
      tiltNeutral(); alt_target = nil; goal = nil; wps = {}
      state = "IDLE"; ui_msg = "stopped"
    elseif c == "set" and #a == 3 then
      ui_msg = setKey(a[2], a[3]) and (a[2] .. " = " .. a[3])
               or "bad key/value (see cfg)"
    elseif c == "cfg" then
      term.setCursorPos(1, 6)
      print(("alt_ceiling=%d  v_max=%.1f  max_tilt=%.2f  pulse=%d"):format(
        cfg.alt_ceiling, cfg.v_max, cfg.max_tilt, cfg.pulse))
      print(("a_brake=%.1f arrive_r=%.1f clearance=%d land_rate=%.1f")
        :format(cfg.a_brake, cfg.arrive_r, cfg.clearance, cfg.land_rate))
      print(("alt kp=%.1f ki=%.1f kd=%.1f | att kp=%.0f ki=%.0f kd=%.0f")
        :format(cfg.alt.kp, cfg.alt.ki, cfg.alt.kd, cfg.att.kp, cfg.att.ki, cfg.att.kd))
    elseif c == "trim" then
      ui_msg = ("trim I=(%.1f, %.1f) rpm - imbalance compensation"):format(att_I.x, att_I.z)
    elseif c == "recal" then
      cfg.s_prime, cfg.h_sign, cfg.rsc_sign, cfg.att_map = nil, nil, nil, nil
      saveCfg(); ui_msg = "calibration reset"
    elseif c == "st" then
      local th = {}
      for _, p in ipairs(props) do
        local okc, t = pcall(p.getThrust); th[#th+1] = okc and ("%.1f"):format(t or 0) or "?"
      end
      ui_msg = ("thrust[%s] asm=%s v=(%.1f %.1f %.1f)"):format(
        table.concat(th, " "), tostring(pa and pa.isAssembled()), est.vx, est.vy, est.vz)
    elseif c == "help" or c == "" then
      term.setCursorPos(1, 6); print(HELP)
    else
      ui_msg = "unknown command. Try: help"
    end
  end
end

-- ---------------- START ----------------
print("Flight controller v3 started. Type: help")
parallel.waitForAny(controlLoop, uiLoop)


-- wget https://raw.githubusercontent.com/ISSSSA/minecraft-data/main/probe.lua probe