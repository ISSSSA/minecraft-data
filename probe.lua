--[[
  kamikaze_v2.lua -- autopilot for a single-use fixed-wing drone (bpla_v2.nbt)
  Create: Avionics 0.5.2 / Simulated 1.3.0 / Aeronautics 1.3.0 / CC 1.120.0

  USAGE
    kamikaze test        diagnostics only, nothing moves
    kamikaze             fly to the marker in the navigation_table

  TARGET: put a lodestone compass (or a map) into the navigation_table.
  There is no coordinate mode anymore: getProjectedSelfPos does not exist
  in the Lua API (verified by decompiling avionics 0.5.2). Guidance runs
  entirely on the nav table's relative readings -- they are sufficient.

  STARTUP ORDER (strict!):
    1. Fuel the engine, light it (flint & steel). With no signal the
       transmission passes rotation fully -- the propeller assembles
       and spins up by itself.
    2. Check thrust direction: if it blows the wrong way, change the
       scroll option (wrench) on the engine or the bearing.
    3. Right-click the physics_assembler to assemble physics.
    4. Run "kamikaze test", then "kamikaze".

  TRANSMISSION NOTE (decompiled AnalogTransmission):
    signal 0  = full rotation passthrough (full throttle)
    signal 15 = full stop
    i.e. the semantics are INVERTED. This script accounts for it:
    setThrottle(15) = full power. We NEVER call releaseSignal(): with no
    redstone nearby the signal falls to 0 and the prop goes full power.
    Stop = forced signal 15.
]]

--=============================== CONFIG =====================================
local CFG = {
  tickRate       = 0.05,
  cruiseAlt      = 90,      -- cruise altitude (getHeight)
  climbAlt       = 60,      -- altitude where climb ends
  terminalRange  = 40,      -- distance to switch into the dive
  fuseRange      = 6,       -- detonate on rangefinder (optical_sensor)
  fuseDistance   = 8,       -- detonate on nav table distance
  armAltitude    = 15,      -- arm above this altitude
  maxThrottle    = 15,      -- 15 = full power (mapped to signal 0 inside)
  cruiseThrottle = 11,
  bombSide       = "back",  -- redstone to the dispensers (computer faces south)
  timeoutSec     = 300,

  -- CONTROL SIGNS. The setManualTarget vector is in the contraption's
  -- LOCAL axes (as built: nose = -Z, bearing faces +Z) and gets clamped
  -- into a 12 deg cone around the bearing axis. Which way the nose
  -- actually goes depends on the prop rotation sign, so the signs are
  -- exposed here:
  --   circles the target / turns away  -> steerSign = -1
  --   noses down on climb / pitches up -> pitchSign = 1
  steerSign = 1,
  pitchSign = -1,   -- default: tail thrust, pushing tail up = nose down

  altKp = 0.020, altKi = 0.0006, altKd = 0.055,
  hdgKp = 0.030, hdgKi = 0.0000, hdgKd = 0.040,
}

--=============================== HELPERS ====================================
local function clamp(v, lo, hi)
  if v < lo then return lo elseif v > hi then return hi else return v end
end

local function call(p, method, ...)
  if not p or type(p[method]) ~= "function" then return nil end
  local r = { pcall(p[method], ...) }
  if not r[1] then return nil end
  return table.unpack(r, 2)
end

local function num(v, fb) return (type(v) == "number" and v) or fb end
local function wrap(a) return ((a + 180) % 360) - 180 end

--=============================== PID ========================================
local PID = {}
PID.__index = PID
function PID.new(kp, ki, kd, iLimit)
  return setmetatable({ kp=kp, ki=ki, kd=kd, i=0, prev=nil,
                        iLimit = iLimit or 50 }, PID)
end
function PID:step(err, dt)
  self.i = clamp(self.i + err * dt, -self.iLimit, self.iLimit)
  local d = 0
  if self.prev then d = (err - self.prev) / dt end
  self.prev = err
  return self.kp * err + self.ki * self.i + self.kd * d
end
function PID:reset() self.i, self.prev = 0, nil end

--=============================== DEVICES ====================================
local D = {
  nav    = peripheral.find("navigation_table"),
  gimbal = peripheral.find("gimbal_sensor"),
  alt    = peripheral.find("altitude_sensor"),
  vel    = peripheral.find("velocity_sensor"),
  optic  = peripheral.find("optical_sensor"),
  asm    = peripheral.find("physics_assembler"),
  engine = peripheral.find("portable_engine"),
  thrust = peripheral.find("gyroscopic_propeller_bearing"),
}
D.throttles = { peripheral.find("analog_transmission") }

--=============================== ACTUATORS ==================================
-- level 0..15, where 15 = full power. The transmission receives 15-level.
local function setThrottle(level)
  level = math.floor(clamp(level, 0, CFG.maxThrottle) + 0.5)
  for _, t in ipairs(D.throttles) do
    call(t, "setSignal", 15 - level)   -- setSignal enables external control itself
  end
  return level
end

-- Full drivetrain stop. NEVER releaseSignal: with no redstone that means full power.
local function stopThrottle()
  for _, t in ipairs(D.throttles) do
    call(t, "setSignal", 15)
  end
end

-- Contraption local axes: nose = -Z, bearing faces +Z.
-- x = sideways, y = up, z along the bearing axis (base +1).
local function setThrustVector(x, y, z)
  if not D.thrust then return end
  local len = math.sqrt(x*x + y*y + z*z)
  if len < 1e-6 then return end
  call(D.thrust, "setManualTarget", { x = x/len, y = y/len, z = z/len })
end

local function neutralThrust()
  if D.thrust then call(D.thrust, "clearManualTarget") end  -- gyro auto-holds against gravity
end

--=============================== TELEMETRY ==================================
local function readState()
  local s = {}
  s.alt    = num(call(D.alt, "getHeight"), 0)
  s.vspeed = num(call(D.alt, "getVerticalSpeed"), 0)
  s.heading = num(call(D.nav, "getHeading"), 0)

  -- all guidance runs on nav table readings (marker required)
  s.dist     = num(call(D.nav, "getDistanceToTarget"), math.huge)
  s.relAngle = num(call(D.nav, "getRelativeAngle"), 0)
  s.vOffset  = num(call(D.nav, "getVerticalOffsetToTarget"), 0)
  s.closure  = num(call(D.nav, "getClosureRate"), 0)

  local ang = call(D.gimbal, "getAngles")
  if type(ang) == "table" then
    s.pitch = num(ang[1], 0); s.roll = num(ang[3], 0)
  else
    s.pitch, s.roll = 0, 0
  end

  -- rangefinder: optical returns range+0.5 on a miss, hence hasHit
  if call(D.optic, "hasHit") then
    s.range = num(call(D.optic, "getDistance"), math.huge)
  else
    s.range = math.huge
  end
  return s
end

--=============================== TEST MODE ==================================
local function testMode()
  print("=== DIAGNOSTICS -- nothing will move ===")
  print()
  local names = peripheral.getNames()
  print("Peripherals on network: " .. #names)
  for _, n in ipairs(names) do
    print(string.format("  %-28s %s", n, peripheral.getType(n)))
  end
  print()
  local req = {
    { "navigation_table",             D.nav,          "CRITICAL" },
    { "analog_transmission",          D.throttles[1], "CRITICAL" },
    { "gyroscopic_propeller_bearing", D.thrust,       "CRITICAL" },
    { "altitude_sensor",              D.alt,          "important" },
    { "gimbal_sensor",                D.gimbal,       "important" },
    { "optical_sensor",               D.optic,        "important (fuse)" },
    { "velocity_sensor",              D.vel,          "optional" },
    { "physics_assembler",            D.asm,          "optional" },
    { "portable_engine",              D.engine,       "optional" },
  }
  for _, r in ipairs(req) do
    print(string.format("  [%s] %-30s %s", r[2] and "ok" or "--", r[1], r[2] and "" or r[3]))
  end
  print()
  print("Physics assembled: " .. tostring(call(D.asm, "isAssembled")))
  print("Mass:              " .. tostring(call(D.asm, "getMass") or "?"))
  print("Engine lit:        " .. tostring(call(D.engine, "isLit")))
  print(string.format("Altitude: %.1f  Heading: %.1f",
        num(call(D.alt,"getHeight"),-1), num(call(D.nav,"getHeading"),0)))
  print("Bearing tilt:      " .. tostring(call(D.thrust, "getTiltAngle")))
  print()
  if call(D.nav, "hasTarget") then
    print(string.format("Target: yes (%s), dist %.0f, rel angle %.1f, dH %.1f",
      tostring(call(D.nav,"getTargetType")),
      num(call(D.nav,"getDistanceToTarget"),-1),
      num(call(D.nav,"getRelativeAngle"),0),
      num(call(D.nav,"getVerticalOffsetToTarget"),0)))
  else
    print("Target: NONE. Put a lodestone compass into the navigation_table.")
  end
  print()
  print("Thrust check: the transmission spins at signal 0 anyway --")
  print("watch which way it blows. Wrong way -> wrench the scroll")
  print("option on the engine or the bearing.")
end

--=============================== WARHEAD ====================================
local armed = false

local function detonate(reason)
  print("STRIKE: " .. reason)
  redstone.setOutput(CFG.bombSide, true)
  sleep(0.3)
  stopThrottle()
  sleep(1.0)
  redstone.setOutput(CFG.bombSide, false)
end

--=============================== ENTRY ======================================
local argv = { ... }

if argv[1] == "test" then
  testMode()
  return
end

if argv[1] then
  print("Coordinate mode is not supported: the nav table has no")
  print("getProjectedSelfPos in the Lua API. Put a lodestone compass")
  print("into the navigation_table and just run: kamikaze")
  return
end

if not D.nav then
  print("FAULT: no navigation_table. Run 'kamikaze test'.")
  return
end
if #D.throttles == 0 then
  print("FAULT: no analog_transmission -- no throttle.")
  return
end
if not D.thrust then
  print("WARNING: no gyroscopic_propeller_bearing -- no steering.")
end
if not call(D.nav, "hasTarget") then
  print("FAULT: no target. Lodestone compass into the navigation_table.")
  return
end
if call(D.asm, "isAssembled") == false then
  print("FAULT: physics not assembled. Right-click the physics_assembler.")
  return
end

print("Target: " .. tostring(call(D.nav, "getTargetType")))
print("Mass:   " .. tostring(call(D.asm, "getMass") or "?"))
print("Launch in 3 s. Ctrl+T to abort.")
sleep(3)

redstone.setOutput(CFG.bombSide, false)

local altPID = PID.new(CFG.altKp, CFG.altKi, CFG.altKd)
local hdgPID = PID.new(CFG.hdgKp, CFG.hdgKi, CFG.hdgKd)
local phase, started, last = "CLIMB", os.clock(), os.clock()

while true do
  local now = os.clock()
  local dt = math.max(now - last, 0.001)
  last = now

  local s = readState()

  if not armed and s.alt > CFG.armAltitude then
    armed = true
    print("ARMED at " .. math.floor(s.alt))
  end

  if phase == "CLIMB" and s.alt >= CFG.climbAlt then
    phase = "CRUISE"; altPID:reset(); print("-> CRUISE")
  elseif phase == "CRUISE" and s.dist <= CFG.terminalRange then
    phase = "TERMINAL"; altPID:reset(); print("-> TERMINAL")
  end

  local throttle, tx, ty, tz = 0, 0, 0, 1

  if phase == "CLIMB" then
    throttle = CFG.maxThrottle
    -- with no override the gyro pulls up by itself (12 deg cone); help a bit
    tx, ty, tz = 0, -0.4 * CFG.pitchSign, 1.0
  elseif phase == "CRUISE" then
    ty = CFG.pitchSign * clamp(altPID:step(CFG.cruiseAlt - s.alt, dt), -0.6, 0.6)
    tx = CFG.steerSign * clamp(hdgPID:step(s.relAngle, dt), -0.8, 0.8)
    tz = 1.0
    throttle = CFG.cruiseThrottle
  else -- TERMINAL
    tx = CFG.steerSign * clamp(s.relAngle / 45, -1, 1)
    ty = CFG.pitchSign * clamp(-s.vOffset / math.max(s.dist, 1), -1, 1)
    tz = 1.0
    throttle = CFG.maxThrottle
  end

  setThrustVector(tx, ty, tz)
  throttle = setThrottle(throttle)

  if armed and phase == "TERMINAL" then
    if s.range <= CFG.fuseRange then
      detonate(string.format("rangefinder %.1f", s.range)); break
    elseif s.dist <= CFG.fuseDistance then
      detonate(string.format("target distance %.1f", s.dist)); break
    end
  end

  if now - started > CFG.timeoutSec then
    print("Timeout. Strike cancelled.")
    stopThrottle(); neutralThrust(); break
  end
  if D.engine and call(D.engine, "isLit") == false then
    print("Engine out.")
    stopThrottle(); break
  end

  term.setCursorPos(1, 12)
  print(string.format("%-9s H=%-6.1f D=%-7.1f rel=%-6.1f dH=%-6.1f clo=%-5.1f thr=%d  ",
        phase, s.alt, s.dist, s.relAngle, s.vOffset, s.closure, throttle))

  sleep(CFG.tickRate)
end

stopThrottle()
print("Program finished.")

-- wget https://raw.githubusercontent.com/ISSSSA/minecraft-data/main/probe.lua probe