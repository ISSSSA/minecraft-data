--[[
  kamikaze.lua -- autopilot for a single-use fixed-wing drone
  Create: Avionics 0.5.2 / Simulated 1.3.0 / Aeronautics 1.3.0 / CC:Tweaked 1.120.0

  Control goes through peripheral METHODS, not redstone:
    analog_transmission:setSignal(0..15)          -- throttle
    gyroscopic_propeller_bearing:setManualTarget  -- thrust vector
    navigation_table                              -- position, target, bearing

  Target comes from the marker item in the navigation table (placed by hand
  in-game), or from the TARGET constant below if the table is empty.

  Phases: CLIMB -> CRUISE -> TERMINAL -> STRIKE
]]

--=============================== CONFIG =====================================
local CFG = {
  tickRate        = 0.05,   -- 20 Hz
  cruiseAlt       = 90,     -- cruise altitude, blocks
  climbAlt        = 60,     -- below this we climb instead of tracking target
  terminalRange   = 40,     -- switch to terminal dive at this range
  fuseRange       = 6,      -- rangefinder sees ground closer than this -> fire
  fuseDistance    = 8,      -- or distance to target below this -> fire
  armAltitude     = 15,     -- warhead arms only above this altitude
  maxThrottle     = 15,
  cruiseThrottle  = 11,
  bombSide        = "back", -- computer face wired to the dispensers
  timeoutSec      = 300,    -- emergency abort
  -- Altitude PID (drives vertical component of the thrust vector)
  altKp = 0.020, altKi = 0.0006, altKd = 0.055,
  -- Heading PID (horizontal steering of the thrust vector)
  hdgKp = 0.030, hdgKi = 0.0000, hdgKd = 0.040,
}

-- Fallback target if the nav table holds no marker. nil = require a marker.
local TARGET = nil   -- example: {x = 1200, y = 70, z = -430}

--=============================== HELPERS ====================================
local function clamp(v, lo, hi)
  if v < lo then return lo elseif v > hi then return hi else return v end
end

-- Avionics peripherals return either a table or several values.
-- Normalise both into a plain array of numbers.
local function toVec(...)
  local n = select("#", ...)
  if n == 0 then return nil end
  local first = select(1, ...)
  if type(first) == "table" then
    return { first.x or first[1], first.y or first[2], first.z or first[3] }
  end
  local t = {}
  for i = 1, n do t[i] = select(i, ...) end
  return t
end

local function call(p, method, ...)
  if not p or type(p[method]) ~= "function" then return nil end
  local r = { pcall(p[method], ...) }
  if not r[1] then return nil end
  return table.unpack(r, 2)
end

local function num(v, fallback)
  return (type(v) == "number" and v) or fallback
end

--=============================== PID ========================================
local PID = {}
PID.__index = PID

function PID.new(kp, ki, kd, iLimit)
  return setmetatable({ kp = kp, ki = ki, kd = kd,
                        i = 0, prev = nil, iLimit = iLimit or 50 }, PID)
end

function PID:step(err, dt)
  self.i = clamp(self.i + err * dt, -self.iLimit, self.iLimit)
  local d = 0
  if self.prev then d = (err - self.prev) / dt end
  self.prev = err
  return self.kp * err + self.ki * self.i + self.kd * d
end

function PID:reset()
  self.i, self.prev = 0, nil
end

--=============================== DEVICE DISCOVERY ===========================
local function findAll()
  local dev = {
    nav    = peripheral.find("navigation_table"),
    gimbal = peripheral.find("gimbal_sensor"),
    alt    = peripheral.find("altitude_sensor"),
    vel    = peripheral.find("velocity_sensor"),
    laser  = peripheral.find("laser_sensor"),
    optic  = peripheral.find("optical_sensor"),
    asm    = peripheral.find("physics_assembler"),
    engine = peripheral.find("portable_engine"),
    thrust = peripheral.find("gyroscopic_propeller_bearing"),
  }
  -- There may be several transmissions; treat them all as one throttle bank.
  dev.throttles = { peripheral.find("analog_transmission") }
  return dev
end

local D = findAll()

local function fail(msg)
  print("FAULT: " .. msg)
  error(msg, 0)
end

if not D.nav then fail("no navigation_table -- nowhere to fly") end
if #D.throttles == 0 then fail("no analog_transmission -- no throttle") end
if not D.thrust then
  print("WARNING: gyroscopic_propeller_bearing not found.")
  print("Thrust vectoring unavailable, throttle-only flight.")
end

--=============================== ACTUATORS ==================================
local function setThrottle(level)
  level = math.floor(clamp(level, 0, CFG.maxThrottle) + 0.5)
  for _, t in ipairs(D.throttles) do
    call(t, "setExternallyControlled", true)
    call(t, "setSignal", level)
  end
  return level
end

local function releaseThrottle()
  for _, t in ipairs(D.throttles) do
    call(t, "setSignal", 0)
    call(t, "releaseSignal")
  end
end

-- Direction is in the craft's body frame:
-- x = right, y = up, z = forward (nose).
local function setThrustVector(x, y, z)
  if not D.thrust then return end
  local len = math.sqrt(x * x + y * y + z * z)
  if len < 1e-6 then return end
  call(D.thrust, "setManualTarget", { x / len, y / len, z / len })
end

--=============================== TARGET =====================================
local function getTarget()
  if call(D.nav, "hasTarget") then
    local t = toVec(call(D.nav, "getTargetPosition"))
    if t and t[1] then return { x = t[1], y = t[2], z = t[3] }, "nav table" end
  end
  if TARGET then return TARGET, "constant" end
  return nil
end

--=============================== TELEMETRY ==================================
local function readState()
  local s = {}
  s.alt      = num(call(D.alt, "getHeight"), 0)
  s.vspeed   = num(call(D.alt, "getVerticalSpeed"), 0)
  s.dist     = num(call(D.nav, "getDistanceToTarget"), math.huge)
  s.relAngle = num(call(D.nav, "getRelativeAngle"), 0)      -- degrees
  s.vOffset  = num(call(D.nav, "getVerticalOffsetToTarget"), 0)
  s.closure  = num(call(D.nav, "getClosureRate"), 0)

  local pos = toVec(call(D.nav, "getProjectedSelfPos"))
  s.pos = pos and { x = pos[1], y = pos[2], z = pos[3] } or nil

  local ang = toVec(call(D.gimbal, "getAngles"))
  s.pitch = ang and num(ang[1], 0) or 0
  s.roll  = ang and num(ang[3], 0) or 0

  -- Rangefinder: laser first, optical sensor as fallback
  s.range = num(call(D.laser, "getClosestHitDistance"), nil)
             or num(call(D.optic, "getDistance"), math.huge)
  return s
end

--=============================== WARHEAD ====================================
local armed = false

local function detonate(reason)
  print("STRIKE: " .. reason)
  redstone.setOutput(CFG.bombSide, true)
  sleep(0.3)
  releaseThrottle()
  setThrustVector(0, 0, 1)
  sleep(1.0)
  redstone.setOutput(CFG.bombSide, false)
end

--=============================== MAIN LOOP ==================================
local altPID = PID.new(CFG.altKp, CFG.altKi, CFG.altKd)
local hdgPID = PID.new(CFG.hdgKp, CFG.hdgKi, CFG.hdgKd)

local target, src = getTarget()
if not target then
  fail("no target: put a marker in the navigation table or set TARGET")
end

print(("Target: %.0f %.0f %.0f  (source: %s)")
      :format(target.x, target.y, target.z, src))
print("Craft mass: " .. tostring(call(D.asm, "getMass") or "?"))
print("Launch in 3 s. Ctrl+T to abort.")
sleep(3)

redstone.setOutput(CFG.bombSide, false)

local phase   = "CLIMB"
local started = os.clock()
local last    = os.clock()

while true do
  local now = os.clock()
  local dt  = math.max(now - last, 0.001)
  last = now

  local s = readState()

  -- Arm only after reaching a safe altitude.
  -- Until armed, detonation cannot happen under any condition.
  if not armed and s.alt > CFG.armAltitude then
    armed = true
    print("ARMED at altitude " .. math.floor(s.alt))
  end

  ---------------------------------------------------------------- phases ---
  if phase == "CLIMB" and s.alt >= CFG.climbAlt then
    phase = "CRUISE"; altPID:reset(); print("-> CRUISE")
  elseif phase == "CRUISE" and s.dist <= CFG.terminalRange then
    phase = "TERMINAL"; altPID:reset(); print("-> TERMINAL")
  end

  ----------------------------------------------------- vector + throttle ---
  local throttle, ty, tz, tx = 0, 0, 1, 0

  if phase == "CLIMB" then
    -- Gain altitude: thrust up and forward, ignore heading for now
    throttle = CFG.maxThrottle
    ty, tz = 0.7, 0.7

  elseif phase == "CRUISE" then
    -- Hold cruise altitude and steer onto the target
    local altErr = CFG.cruiseAlt - s.alt
    ty = clamp(altPID:step(altErr, dt), -0.6, 0.6)
    local hdgErr = ((s.relAngle + 180) % 360) - 180   -- wrap to -180..180
    tx = clamp(hdgPID:step(hdgErr, dt), -0.8, 0.8)
    tz = 1.0
    throttle = CFG.cruiseThrottle

  elseif phase == "TERMINAL" then
    -- Dive: aim precisely, vertical component from height offset to target
    local hdgErr = ((s.relAngle + 180) % 360) - 180
    tx = clamp(hdgErr / 45, -1, 1)
    ty = clamp(s.vOffset / math.max(s.dist, 1), -1, 1)
    tz = 1.0
    throttle = CFG.maxThrottle
  end

  setThrustVector(tx, ty, tz)
  throttle = setThrottle(throttle)

  ------------------------------------------------------------------ fuse ---
  if armed and phase == "TERMINAL" then
    if s.range <= CFG.fuseRange then
      detonate(("rangefinder %.1f"):format(s.range)); break
    elseif s.dist <= CFG.fuseDistance then
      detonate(("target distance %.1f"):format(s.dist)); break
    end
  end

  --------------------------------------------------------------- aborts ----
  if now - started > CFG.timeoutSec then
    print("Timeout. Throttle to zero, strike cancelled.")
    releaseThrottle(); setThrustVector(0, 1, 0); break
  end
  if D.engine and call(D.engine, "isLit") == false then
    print("Engine out -- no fuel left.")
    releaseThrottle(); break
  end

  ------------------------------------------------------------ telemetry ----
  term.setCursorPos(1, 10)
  print(("%-9s H=%-6.1f D=%-7.1f dH=%-6.1f roll=%-5.1f thr=%d  ")
        :format(phase, s.alt, s.dist, s.vOffset, s.roll, throttle))

  sleep(CFG.tickRate)
end

releaseThrottle()
print("Program finished.")