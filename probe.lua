-- ============================================================
--  AUTONOMOUS DRONE  |  Create Aeronautics + Avionics + CC:Tweaked
--  Flies to arbitrary coordinates without a pilot.
--  Positioning: navigation_table + lodestone compass (anchor),
--  altitude: altitude_sensor, stability: gimbal + gyro propellers.
--  Save as startup.lua on the onboard computer.
-- ============================================================

-- ---------------- CONFIG ----------------
-- Change at runtime with:  set <key> <value>   (e.g. "set alt_ceiling 60",
-- "set alt.kp 25"). Values persist in autopilot.cfg.
local CFG_FILE = "autopilot.cfg"
local cfg = {
  anchor   = { x = 0, y = 64, z = 0 }, -- LODESTONE coords. Set with: anchor <x> <y> <z>
  alt_ceiling = 30, -- max height above launch point, blocks  <-- your limit
  s_prime  = nil,   -- bearing sign            (auto-calibrated)
  h_sign   = nil,   -- body frame handedness   (auto-calibrated)
  motor_sign = nil, -- motor sign for upthrust (auto-calibrated)
  ground_y = nil,   -- launch altitude, captured on arming
  max_tilt = 0.20,  -- horizontal thrust component (prop cone ~12 deg => max 0.21)
  tilt_slew = 0.05, -- max tilt change per control tick (smoothness)
  v_max    = 9,     -- cruise speed cap, blocks/s
  a_brake  = 2.0,   -- braking decel used for approach curve, blocks/s^2
  arrive_r = 2.5,   -- arrival radius, blocks
  clearance = 8,    -- transit altitude margin above start/goal
  land_rate = 1.6,  -- descent speed while landing, blocks/s
  alt = { kp = 22, ki = 3.5, kd = 30 },   -- altitude PID -> motor speed
  hor = { kv = 0.045, kd = 0.012 },       -- velocity P / accel D -> tilt
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
local motor = peripheral.find("Create_CreativeMotor")
local nav   = peripheral.find("navigation_table")
local alt   = peripheral.find("altitude_sensor")
local gyro  = peripheral.find("gimbal_sensor")
local pa    = peripheral.find("physics_assembler")

local function need(p, n)
  if not p then error(("Peripheral '%s' not found. Check cables/modems."):format(n), 0) end
end
if #props ~= 4 then
  error("Found " .. #props .. "/4 propellers. Check the wired network.", 0)
end
need(motor, "Create_CreativeMotor"); need(nav, "navigation_table")
need(alt, "altitude_sensor"); need(gyro, "gimbal_sensor")

-- ---------------- STATE ----------------
local state = "IDLE"       -- IDLE|ARM|CAL|TAKEOFF|NAV|HOVER|LAND
local goal = nil           -- {x,y,z, land=bool}
local wps = {}             -- waypoint queue
local est = { x=0, z=0, y=0, vx=0, vz=0, vy=0, ok=false }
local ui_msg = "type a command (help)"
local alt_target, alt_I = nil, 0
local cur_bx, cur_bz = 0, 0            -- current tilt command (for slew limiting)
local DT = 0.15                        -- control period, s
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
local function rawTilt(bx, bz)
  local m = math.sqrt(bx*bx + bz*bz)
  if m > cfg.max_tilt then bx, bz = bx*cfg.max_tilt/m, bz*cfg.max_tilt/m end
  for _, p in ipairs(props) do pcall(p.setManualTarget, { bx, 1, bz }) end
  cur_bx, cur_bz = bx, bz
end
local function setTilt(bx, bz)          -- with slew limiting
  local dx, dz = bx - cur_bx, bz - cur_bz
  local d = math.sqrt(dx*dx + dz*dz)
  if d > cfg.tilt_slew then
    bx = cur_bx + dx / d * cfg.tilt_slew
    bz = cur_bz + dz / d * cfg.tilt_slew
  end
  rawTilt(bx, bz)
end
local function tiltNeutral()            -- gyros hold "world up" by themselves
  allProps("clearManualTarget"); cur_bx, cur_bz = 0, 0
end
local function setMotor(v)
  pcall(motor.setGeneratedSpeed, clamp(math.floor(v + 0.5), -256, 256))
end

-- ---------------- NAVIGATION ----------------
-- World azimuth convention: az(dir)=atan2(dx,dz); dir(az)=(sin az, cos az). Degrees.
local function readSensors()
  local h = alt.getHeight()
  est.vy = last_h and (h - last_h) / DT or 0
  last_h = h
  est.y = h

  if not nav.hasTarget() then est.ok = false; return end
  local d3   = nav.getDistanceToTarget()
  local dyA  = nav.getVerticalOffsetToTarget()          -- anchor.y - self.y
  local brg  = nav.getBearing()                         -- target vs nose, [-180,180]
  local hdg  = nav.getHeading()                         -- nose heading in world
  local dh   = math.sqrt(math.max(d3*d3 - dyA*dyA, 0))
  local sp, hs = cfg.s_prime or 1, cfg.h_sign or 1
  local azA = hdg + hs * sp * brg                       -- world azimuth to anchor
  local sx = cfg.anchor.x - dh * math.sin(math.rad(azA))
  local sz = cfg.anchor.z - dh * math.cos(math.rad(azA))
  local vx = last_x and (sx - last_x) / DT or 0
  local vz = last_z and (sz - last_z) / DT or 0
  est.vx = est.vx * 0.7 + vx * 0.3                      -- EMA smoothing
  est.vz = est.vz * 0.7 + vz * 0.3
  last_x, last_z = sx, sz
  est.x, est.z = sx, sz
  est.hdg, est.brg, est.dh = hdg, brg, dh
  est.ok = true
end

-- rotate a WORLD-frame horizontal vector into BODY frame -> (right, forward)
local function worldToBody(wx, wz)
  local hd = math.rad(est.hdg or 0)
  local along = wx * math.sin(hd) + wz * math.cos(hd)   -- projection on nose axis
  local lat   = wx * math.cos(hd) - wz * math.sin(hd)   -- lateral component
  return (cfg.h_sign or 1) * lat, along
end

-- ---------------- ALTITUDE LOOP ----------------
local function altitudeLoop()
  if not alt_target then return end
  alt_target = math.min(alt_target, ceilingY())         -- hard altitude ceiling
  local e = alt_target - est.y
  local out = alt_I + cfg.alt.kp * e - cfg.alt.kd * est.vy
  -- anti-windup: only integrate while the motor is not saturated
  if out > 0 and out < 256 then
    alt_I = clamp(alt_I + e * cfg.alt.ki * DT, -256, 256)
  end
  setMotor((cfg.motor_sign or 1) * clamp(out, 0, 256))
end

-- ---------------- HORIZONTAL CONTROLLER ----------------
-- cascade: position -> desired velocity (braking curve) -> velocity PD -> tilt
local prev_evx, prev_evz = 0, 0
local function flyToward(tx, tz, speed_cap)
  local dx, dz = tx - est.x, tz - est.z
  local dist = math.sqrt(dx*dx + dz*dz)
  if dist < 0.05 then setTilt(0, 0); return dist end
  local v_des = math.min(speed_cap or cfg.v_max,
                         math.sqrt(2 * cfg.a_brake * math.max(dist - cfg.arrive_r * 0.5, 0)))
  local vdx, vdz = dx / dist * v_des, dz / dist * v_des
  local evx, evz = vdx - est.vx, vdz - est.vz           -- velocity error (world)
  local ax = cfg.hor.kv * evx + cfg.hor.kd * (evx - prev_evx) / DT
  local az = cfg.hor.kv * evz + cfg.hor.kd * (evz - prev_evz) / DT
  prev_evx, prev_evz = evx, evz
  local bx, bz = worldToBody(ax, az)
  setTilt(bx, bz)
  return dist
end

-- ---------------- CALIBRATION ----------------
local function calibrateThrust()
  ui_msg = "CAL: thrust direction..."
  alt_target = nil
  for _, sgn in ipairs({ 1, -1 }) do
    local h0 = alt.getHeight()
    setMotor(sgn * 160); sleep(2.5)
    local dh = alt.getHeight() - h0
    setMotor(sgn * 60)                       -- keep soft support, do not cut
    if dh > 0.5 then cfg.motor_sign = sgn; saveCfg(); return true end
    setMotor(0); sleep(1.5)
  end
  return false
end

local function calibrateBearing()
  ui_msg = "CAL: bearing sign..."
  for _, sp in ipairs({ 1, -1 }) do
    cfg.s_prime = sp
    local d0 = nav.getDistanceToTarget()
    local t0 = os.clock()
    while os.clock() - t0 < 4 do             -- tilt straight at the anchor
      readSensors(); altitudeLoop()
      local b = math.rad(sp * nav.getBearing())
      rawTilt(math.sin(b) * 0.10, math.cos(b) * 0.10)
      sleep(DT)
    end
    tiltNeutral()
    if nav.getDistanceToTarget() < d0 - 1.0 then saveCfg(); return true end
    sleep(2)                                  -- bleed off speed
  end
  return false
end

local function calibrateFrame()
  ui_msg = "CAL: body frame..."
  readSensors()
  -- hypotheses h=+-1 give mirrored positions; move strictly "forward" (beta=0):
  -- the correct hypothesis makes displacement azimuth match the heading
  local snap = {}
  for _, hh in ipairs({ 1, -1 }) do
    cfg.h_sign = hh; readSensors()
    snap[hh] = { x = est.x, z = est.z }
  end
  local t0 = os.clock()
  while os.clock() - t0 < 4 do
    readSensors(); altitudeLoop()
    rawTilt(0, 0.10)                          -- body forward
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

-- ---------------- MAIN LOOP ----------------
local function safetyOK()
  local a = gyro.getAngles()
  local tilt = math.max(math.abs(a[1] or 0), math.abs(a[2] or 0))
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
      cfg.ground_y = est.y; saveCfg()          -- ceiling reference = launch altitude
      allProps("assemble")
      ui_msg = "rotors assembled. Click the Physics Assembler!"
      if not pa or pa.isAssembled() then state = "CAL" end

    elseif state == "CAL" then
      if not est.ok then
        ui_msg = "no target in nav table! Insert a lodestone compass."
      else
        if not cfg.motor_sign and not calibrateThrust() then
          ui_msg = "thrust cannot lift the craft - check rotors"; state = "IDLE"
        else
          alt_target = math.min(est.y + 6, ceilingY())
          if not cfg.s_prime then calibrateBearing() end
          if not cfg.h_sign  then calibrateFrame()   end
          ui_msg = "calibration done"; state = goal and "TAKEOFF" or "HOVER"
        end
      end

    elseif state == "TAKEOFF" then
      local transit = math.max(est.y, goal.y) + cfg.clearance
      alt_target = math.min(transit, ceilingY())
      if math.abs(est.y - alt_target) < 2 then state = "NAV" end
      altitudeLoop()

    elseif state == "NAV" then
      altitudeLoop()
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
      altitudeLoop()
      if est.ok and safetyOK() and goal then
        flyToward(goal.x, goal.z, 2.0)         -- gentle position hold
      end

    elseif state == "LAND" then
      safetyOK()
      alt_target = (alt_target or est.y) - cfg.land_rate * DT
      altitudeLoop()
      -- commanded well below yet not descending => touched down
      if est.vy > -0.08 and (alt_target < est.y - 3) then
        setMotor(0); tiltNeutral(); alt_I = 0; alt_target = nil
        if not nextGoal() then state = "IDLE"; ui_msg = "landed" end
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
 wp <x> <y> <z>      queue a waypoint  (wp clear / wp list)
 go [land]           fly the waypoint queue
 home                fly to the anchor
 hover / land / stop
 set <key> <value>   change config (set alt_ceiling 60, set alt.kp 25)
 cfg                 show config     recal: reset calibration
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
      setMotor(0); tiltNeutral(); alt_target = nil; goal = nil; wps = {}
      state = "IDLE"; ui_msg = "stopped"
    elseif c == "set" and #a == 3 then
      ui_msg = setKey(a[2], a[3]) and (a[2] .. " = " .. a[3])
               or "bad key/value (see cfg)"
    elseif c == "cfg" then
      term.setCursorPos(1, 6)
      print(("alt_ceiling=%d  v_max=%.1f  max_tilt=%.2f"):format(
        cfg.alt_ceiling, cfg.v_max, cfg.max_tilt))
      print(("a_brake=%.1f  arrive_r=%.1f  clearance=%d  land_rate=%.1f")
        :format(cfg.a_brake, cfg.arrive_r, cfg.clearance, cfg.land_rate))
      print(("alt.kp=%.1f ki=%.1f kd=%.1f  hor.kv=%.3f kd=%.3f")
        :format(cfg.alt.kp, cfg.alt.ki, cfg.alt.kd, cfg.hor.kv, cfg.hor.kd))
    elseif c == "recal" then
      cfg.s_prime, cfg.h_sign, cfg.motor_sign = nil, nil, nil; saveCfg()
      ui_msg = "calibration reset"
    elseif c == "st" then
      ui_msg = ("props=%d asm=%s v=(%.1f %.1f %.1f)"):format(
        #props, tostring(pa and pa.isAssembled()), est.vx, est.vy, est.vz)
    elseif c == "help" or c == "" then
      term.setCursorPos(1, 6); print(HELP)
    else
      ui_msg = "unknown command. Try: help"
    end
  end
end

-- ---------------- START ----------------
allProps("setThrustHandedness", "right_handed")  -- same blade pitch on all rotors
print("Autopilot started. Type: help")
parallel.waitForAny(controlLoop, uiLoop)


-- wget https://raw.githubusercontent.com/ISSSSA/minecraft-data/main/probe.lua probe