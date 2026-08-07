--[[
  kamikaze.lua -- autopilot for a single-use fixed-wing drone
  Create: Avionics 0.5.2 / Simulated 1.3.0 / Aeronautics 1.3.0 / CC 1.120.0

  USAGE
    kamikaze test            diagnostics only, no flight, nothing moves
    kamikaze 1200 70 -430    fly to these world coordinates
    kamikaze                 fly to the marker in the navigation table

  Coordinates are resolved by the script itself from getProjectedSelfPos,
  so a lodestone marker is optional.
]]

--=============================== CONFIG =====================================
local CFG = {
  tickRate       = 0.05,
  cruiseAlt      = 90,
  climbAlt       = 60,
  terminalRange  = 40,
  fuseRange      = 6,
  fuseDistance   = 8,
  armAltitude    = 15,
  maxThrottle    = 15,
  cruiseThrottle = 11,
  bombSide       = "back",
  timeoutSec     = 300,
  -- Add this to every heading reading. If the drone circles the target
  -- instead of closing on it, try 90 / 180 / 270 here. Run "kamikaze test"
  -- to see raw heading vs computed bearing side by side.
  headingOffset  = 0,
  altKp = 0.020, altKi = 0.0006, altKd = 0.055,
  hdgKp = 0.030, hdgKi = 0.0000, hdgKd = 0.040,
}

--=============================== HELPERS ====================================
local function clamp(v, lo, hi)
  if v < lo then return lo elseif v > hi then return hi else return v end
end

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

local function num(v, fb) return (type(v) == "number" and v) or fb end

-- Wrap an angle into -180..180
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
  laser  = peripheral.find("laser_sensor"),
  asm    = peripheral.find("physics_assembler"),
  engine = peripheral.find("portable_engine"),
  thrust = peripheral.find("gyroscopic_propeller_bearing"),
}
D.throttles = { peripheral.find("analog_transmission") }

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

-- Body frame: x = right, y = up, z = forward (nose)
local function setThrustVector(x, y, z)
  if not D.thrust then return end
  local len = math.sqrt(x*x + y*y + z*z)
  if len < 1e-6 then return end
  call(D.thrust, "setManualTarget", { x/len, y/len, z/len })
end

--=============================== NAVIGATION =================================
-- Own position in WORLD coordinates. This is what makes coordinate
-- targeting possible at all -- the computer lives in a sub-level and
-- its raw position means nothing outside the contraption.
local function selfPos()
  local p = toVec(call(D.nav, "getProjectedSelfPos"))
  if p and type(p[1]) == "number" then
    return { x = p[1], y = p[2], z = p[3] }
  end
  return nil
end

local function heading()
  return num(call(D.nav, "getHeading"), 0) + CFG.headingOffset
end

-- Minecraft yaw convention: 0 = +Z (south), 90 = -X (west), 180 = -Z (north)
local function bearingTo(from, to)
  return math.deg(math.atan2(-(to.x - from.x), to.z - from.z))
end

local function horizDist(from, to)
  local dx, dz = to.x - from.x, to.z - from.z
  return math.sqrt(dx*dx + dz*dz)
end

--=============================== TELEMETRY ==================================
local TARGET = nil        -- filled from argv or from the nav table

local function readState()
  local s = {}
  s.alt    = num(call(D.alt, "getHeight"), 0)
  s.vspeed = num(call(D.alt, "getVerticalSpeed"), 0)
  s.pos    = selfPos()
  s.heading = heading()

  if TARGET and s.pos then
    -- Computed by us, no marker required
    s.dist     = horizDist(s.pos, TARGET)
    s.bearing  = bearingTo(s.pos, TARGET)
    s.relAngle = wrap(s.bearing - s.heading)
    s.vOffset  = TARGET.y - s.pos.y
  else
    -- Fall back to the nav table's own readings
    s.dist     = num(call(D.nav, "getDistanceToTarget"), math.huge)
    s.relAngle = num(call(D.nav, "getRelativeAngle"), 0)
    s.vOffset  = num(call(D.nav, "getVerticalOffsetToTarget"), 0)
    s.bearing  = num(call(D.nav, "getBearing"), 0)
  end

  local ang = toVec(call(D.gimbal, "getAngles"))
  s.pitch = ang and num(ang[1], 0) or 0
  s.roll  = ang and num(ang[3], 0) or 0

  s.range = num(call(D.laser, "getClosestHitDistance"), nil)
             or num(call(D.optic, "getDistance"), math.huge)
  return s
end

--=============================== TEST MODE ==================================
local function testMode()
  print("=== DIAGNOSTICS -- nothing will move ===")
  print()
  local names = peripheral.getNames()
  print("Peripherals attached: " .. #names)
  for _, n in ipairs(names) do
    print(string.format("  %-24s %s", n, peripheral.getType(n)))
  end
  print()

  local required = {
    { "navigation_table",             D.nav,           "CRITICAL" },
    { "analog_transmission",          D.throttles[1],  "CRITICAL" },
    { "gyroscopic_propeller_bearing", D.thrust,        "CRITICAL" },
    { "gimbal_sensor",                D.gimbal,        "important" },
    { "altitude_sensor",              D.alt,           "important" },
    { "optical_sensor",               D.optic,         "optional" },
    { "velocity_sensor",              D.vel,           "optional" },
    { "physics_assembler",            D.asm,           "optional" },
    { "portable_engine",              D.engine,        "optional" },
  }
  for _, r in ipairs(required) do
    print(string.format("  [%s] %-30s %s",
          r[2] and "ok" or "--", r[1], r[2] and "" or r[3]))
  end
  print()

  local p = selfPos()
  if p then
    print(string.format("World position: %.1f %.1f %.1f", p.x, p.y, p.z))
  else
    print("World position: UNAVAILABLE (getProjectedSelfPos returned nothing)")
  end
  print(string.format("Heading:  %.1f  (offset %d applied)",
        heading(), CFG.headingOffset))
  print(string.format("Altitude: %.1f", num(call(D.alt, "getHeight"), -1)))
  print("Mass:     " .. tostring(call(D.asm, "getMass") or "?"))
  print()
  print("Compare 'World position' with F3 in game.")
  print("If they match, coordinate targeting will work.")
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

--=============================== ENTRY ======================================
local argv = { ... }

if argv[1] == "test" then
  testMode()
  return
end

if not D.nav then
  print("FAULT: no navigation_table")
  print("Run 'kamikaze test' to see what is attached.")
  return
end
if #D.throttles == 0 then
  print("FAULT: no analog_transmission -- no throttle")
  return
end
if not D.thrust then
  print("WARNING: no gyroscopic_propeller_bearing, no steering available")
end

-- Target: from the command line, else from the nav table marker
if argv[1] then
  local x, y, z = tonumber(argv[1]), tonumber(argv[2]), tonumber(argv[3])
  if not (x and y and z) then
    print("Usage: kamikaze <x> <y> <z>   |   kamikaze test   |   kamikaze")
    return
  end
  TARGET = { x = x, y = y, z = z }
  if not selfPos() then
    print("FAULT: getProjectedSelfPos unavailable, cannot navigate by coords.")
    print("Use a lodestone compass in the navigation table instead.")
    return
  end
  print(string.format("Target from command line: %.0f %.0f %.0f", x, y, z))
elseif call(D.nav, "hasTarget") then
  local t = toVec(call(D.nav, "getTargetPosition"))
  if t and type(t[1]) == "number" then TARGET = { x=t[1], y=t[2], z=t[3] } end
  print("Target from navigation table marker")
else
  print("FAULT: no target.")
  print("Either: kamikaze <x> <y> <z>")
  print("Or:     put a lodestone compass in the navigation table")
  return
end

print("Mass: " .. tostring(call(D.asm, "getMass") or "?"))
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
    ty, tz = 0.7, 0.7
  elseif phase == "CRUISE" then
    ty = clamp(altPID:step(CFG.cruiseAlt - s.alt, dt), -0.6, 0.6)
    tx = clamp(hdgPID:step(s.relAngle, dt), -0.8, 0.8)
    tz = 1.0
    throttle = CFG.cruiseThrottle
  else
    tx = clamp(s.relAngle / 45, -1, 1)
    ty = clamp(s.vOffset / math.max(s.dist, 1), -1, 1)
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
    releaseThrottle(); setThrustVector(0, 1, 0); break
  end
  if D.engine and call(D.engine, "isLit") == false then
    print("Engine out.")
    releaseThrottle(); break
  end

  term.setCursorPos(1, 12)
  print(string.format("%-9s H=%-6.1f D=%-7.1f rel=%-6.1f dH=%-6.1f thr=%d  ",
        phase, s.alt, s.dist, s.relAngle, s.vOffset, throttle))

  sleep(CFG.tickRate)
end

releaseThrottle()
print("Program finished.")
