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
  motor_direction = -1, -- unified rotation direction: -1 = reverse, +1 = forward
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

-- ---------------- DIRECTION MIGRATION ----------------
-- Older versions stored the RSC direction separately. That could make the
-- Creative Motor and RSCs fight each other. From v4 onward one sign controls
-- the complete drivetrain. For this aircraft the required direction is -1.
if cfg.motor_direction ~= -1 and cfg.motor_direction ~= 1 then
  cfg.motor_direction = -1
end
cfg.rsc_sign = nil

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

-- ---------------- MOTOR / RSC CONTROL ----------------
-- Creative Motor is the source of rotation. RSCs are the speed controllers.
-- They must always use the SAME direction sign.
--
-- Important: do not derive the Creative Motor speed from PID corrections.
-- The source stays at a small, fixed speed (-30 by default), while the RSCs
-- request the actual rotor RPM. This matches the original working architecture
-- and changes only the direction handling.

local MOTOR_SOURCE_RPM = 30

local function motorSetMagnitude(magnitude)
  local dir = (cfg.motor_direction or -1) < 0 and -1 or 1
  local mag = clamp(math.abs(magnitude or 0), 0, 256)
  if mag < 0.5 then
    pcall(motor.setGeneratedSpeed, 0)
  else
    pcall(motor.setGeneratedSpeed, dir * math.floor(mag + 0.5))
  end
end

local function motorStart()
  motorSetMagnitude(MOTOR_SOURCE_RPM)
end

local function motorStop()
  -- Direct peripheral call; this function never calls itself.
  pcall(motor.setGeneratedSpeed, 0)
end

-- per-rotor RPM: collective (altitude) + differential (attitude) + mapping pulse
local function applyRotors(base, cx, cz)
  local dir = (cfg.motor_direction or -1) < 0 and -1 or 1

  for _, r in ipairs(rscs) do
    local nm = peripheral.getName(r)
    local d = cfg.att_map and cfg.att_map[nm]
    local corr = d and (cx * d.x + cz * d.z) or 0
    if r == pulse_rsc then corr = corr + pulse_amt end

    -- PID only changes magnitude. The sign is applied exactly once here.
    local magnitude = clamp(base + corr, 0, 256)
    local target = dir * math.floor(magnitude + 0.5)
    pcall(r.setTargetSpeed, target)
  end

  -- Keep the source alive whenever rotor control requests non-zero RPM.
  if base > 0.5 or math.abs(cx) > 0.5 or math.abs(cz) > 0.5 or pulse_amt > 0 then
    motorStart()
  end
end

local function rotorsOff()
  for _, r in ipairs(rscs) do
    pcall(r.setTargetSpeed, 0)
  end
  motorStop()
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
-- 1) ground: normalize blade pitch so EVERY rotor thrusts UP.
--    Works for any spin direction/gearbox parity: reads the signed
--    getThrust() of each bearing and flips its blade pitch if negative.
local function readThrusts()
  local t = {}
  for i, p in ipairs(props) do
    local okc, th = pcall(p.getThrust)
    t[i] = (okc and th) or 0
  end
  return t
end

local function calibrateBlades()
  ui_msg = "CAL: spooling rotors..."
  applyRotors(100, 0, 0); sleep(2.5)
  local th = readThrusts()
  local flipped = 0
  for i, p in ipairs(props) do
    if th[i] < -0.01 then
      local h = p.getThrustHandedness()
      pcall(p.setThrustHandedness,
            h == "right_handed" and "left_handed" or "right_handed")
      flipped = flipped + 1
    end
  end
  if flipped > 0 then sleep(1.5); th = readThrusts() end
  local bad = 0
  for i = 1, #th do if th[i] <= 0.01 then bad = bad + 1 end end
  ui_msg = ("blades: [%s] flipped=%d"):format(
    table.concat({("%.1f"):format(th[1] or 0), ("%.1f"):format(th[2] or 0),
                  ("%.1f"):format(th[3] or 0), ("%.1f"):format(th[4] or 0)}, " "), flipped)
  return bad == 0, th
end

-- 2) ground->air: collective climb check (signs are already normalized per prop)
local function liftCheck()
  ui_msg = "CAL: lift check..."
  local h0 = alt.getHeight()
  applyRotors(190, 0, 0); sleep(3.0)
  local dh = alt.getHeight() - h0
  applyRotors(80, 0, 0)
  return dh > 0.5, dh
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
local tilt_since = nil
local function safetyOK()
  local tx, tz = gravityTilt()
  local tilt = math.deg(math.asin(clamp(math.sqrt(tx*tx + tz*tz), 0, 1)))
  if tilt > 40 then
    tiltNeutral(); ui_msg = ("TILT %.0f deg! recovering..."):format(tilt)
    tilt_since = tilt_since or os.clock()
    if os.clock() - tilt_since > 4 then      -- cannot recover: emergency landing
      goal = nil; wps = {}; state = "LAND"
      ui_msg = "unrecoverable tilt - emergency landing"
    end
    return false
  end
  tilt_since = nil
  return true
end

-- ground pre-flight diagnostic (command: test)
local function preflight()
  local lines = {}
  local function chk(okc, msg) lines[#lines+1] = (okc and "PASS " or "FAIL ") .. msg end
  chk(#props == 4, "propellers on network: " .. #props .. "/4")
  chk(#rscs == 4, "speed controllers: " .. #rscs .. "/4")
  chk(motor ~= nil, "creative motor")
  chk(nav ~= nil and alt ~= nil and gyro ~= nil, "nav table + altimeter + gyro")
  chk(pa ~= nil, "physics assembler on network")
  chk(nav and nav.hasTarget() or false, "compass in nav table")
  local g = gyro and gyro.getGravity() or {0,0,0}
  local gm = math.sqrt((g[1] or 0)^2 + (g[2] or 0)^2 + (g[3] or 0)^2)
  chk(gm > 0.5, ("gyro reads gravity (%.2f)"):format(gm))
  motorStart()
  allProps("assemble")
  applyRotors(80, 0, 0); sleep(2.5)
  local spun = 0
  for _, p in ipairs(props) do
    local okc, s = pcall(p.getRotationSpeed)
    if okc and math.abs(s or 0) > 5 then spun = spun + 1 end
  end
  chk(spun == 4, "rotors spinning from drivetrain: " .. spun .. "/4")
  local bl_ok, th = calibrateBlades()
  chk(bl_ok, ("all rotors thrust UP [%s]"):format(table.concat({
    ("%.1f"):format(th[1] or 0), ("%.1f"):format(th[2] or 0),
    ("%.1f"):format(th[3] or 0), ("%.1f"):format(th[4] or 0)}, " ")))
  rotorsOff()
  term.setCursorPos(1, 6)
  for _, l in ipairs(lines) do print(l) end
  ui_msg = "pre-flight check finished"
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
      motorStart()    -- constant source; RSCs do the mixing
      allProps("assemble")
      ui_msg = "rotors assembled. Click the Physics Assembler!"
      if not pa or pa.isAssembled() then state = "CAL" end

    elseif state == "CAL" then
      if not est.ok then
        ui_msg = "no target in nav table! Insert a lodestone compass."
      else
        local blades_ok = calibrateBlades()
        local lifts, dh = false, 0
        if blades_ok then lifts, dh = liftCheck() end
        if not blades_ok then
          ui_msg = "some rotors give no thrust - check sails/assembly (st)"
          rotorsOff(); state = "IDLE"
        elseif not lifts then
          ui_msg = ("thrust too low (dh=%.1f) - check sails or reduce weight"):format(dh)
          rotorsOff(); state = "IDLE"
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
        rotorsOff()
        tiltNeutral(); alt_I = 0; alt_target = nil
        att_I.x, att_I.z = 0, 0
        if nextGoal() then motorStart()
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
 arm                 spin up + calibrate + hover
 test                ground pre-flight diagnostic
 disasm              fold rotors (on the ground)
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
    elseif c == "arm" then
      goal = nil; wps = {}
      if state == "IDLE" then state = "ARM"; ui_msg = "arming..." end
    elseif c == "hover" then
      goal = nil; wps = {}; tiltNeutral()
      if state == "IDLE" then state = "ARM"; ui_msg = "arming, will hover"
      else alt_target = est.y; state = "HOVER" end
    elseif c == "land" then
      if state ~= "IDLE" then
        goal = nil; wps = {}; tiltNeutral(); state = "LAND"
      else ui_msg = "not flying" end
    elseif c == "stop" then
      rotorsOff()
      tiltNeutral(); alt_target = nil; goal = nil; wps = {}
      state = "IDLE"; ui_msg = "stopped"
    elseif c == "set" and #a == 3 then
      ui_msg = setKey(a[2], a[3]) and (a[2] .. " = " .. a[3])
               or "bad key/value (see cfg)"
    elseif c == "cfg" then
      term.setCursorPos(1, 6)
      print(("alt_ceiling=%d  v_max=%.1f  max_tilt=%.2f  pulse=%d  motor_dir=%d"):format(
        cfg.alt_ceiling, cfg.v_max, cfg.max_tilt, cfg.pulse, cfg.motor_direction or -1))
      print(("a_brake=%.1f arrive_r=%.1f clearance=%d land_rate=%.1f")
        :format(cfg.a_brake, cfg.arrive_r, cfg.clearance, cfg.land_rate))
      print(("alt kp=%.1f ki=%.1f kd=%.1f | att kp=%.0f ki=%.0f kd=%.0f")
        :format(cfg.alt.kp, cfg.alt.ki, cfg.alt.kd, cfg.att.kp, cfg.att.ki, cfg.att.kd))
    elseif c == "trim" then
      ui_msg = ("trim I=(%.1f, %.1f) rpm - imbalance compensation"):format(att_I.x, att_I.z)
    elseif c == "test" then
      if state == "IDLE" then preflight()
      else ui_msg = "test only on the ground (stop first)" end
    elseif c == "disasm" then
      if state == "IDLE" then
        rotorsOff()
        allProps("disassemble"); ui_msg = "rotors disassembled"
      else ui_msg = "only on the ground (stop first)" end
    elseif c == "recal" then
      cfg.s_prime, cfg.h_sign, cfg.att_map = nil, nil, nil
      saveCfg(); ui_msg = "calibration reset (blade pitch persists in bearings)"
    elseif c == "st" then
      local th, rpm = {}, {}
      for _, p in ipairs(props) do
        local okc, t = pcall(p.getThrust); th[#th+1] = okc and ("%.1f"):format(t or 0) or "?"
        local okr, r = pcall(p.getRotationSpeed); rpm[#rpm+1] = okr and ("%.0f"):format(r or 0) or "?"
      end
      ui_msg = ("thr[%s] rpm[%s] asm=%s vy=%.1f"):format(
        table.concat(th, " "), table.concat(rpm, " "),
        tostring(pa and pa.isAssembled()), est.vy)
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