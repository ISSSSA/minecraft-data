--[[
  probe.lua — разведка бортовых систем
  Create Aeronautics 1.3.0 / Simulated / CC:Tweaked 1.120.0

  Запускать НА БОРТУ собранной контрапции.
  Выясняет: какая периферия видна, что реально возвращают её методы,
  работает ли gps.locate() внутри суб-мира.

  Результат пишется в /probe.log — забери его через docking_connector.
]]

local LOG = "/probe.log"
local log = fs.open(LOG, "w")

local function out(s)
  print(s)
  log.writeLine(s)
end

-- Разворачивает любое возвращаемое значение в читаемую строку,
-- потому что заранее неизвестно, таблица это или набор значений.
local function fmt(...)
  local n = select("#", ...)
  if n == 0 then return "<nil/void>" end
  local parts = {}
  for i = 1, n do
    local v = select(i, ...)
    if type(v) == "table" then
      local kv = {}
      for k, x in pairs(v) do kv[#kv + 1] = tostring(k) .. "=" .. tostring(x) end
      table.sort(kv)
      parts[#parts + 1] = "{" .. table.concat(kv, ", ") .. "}"
    else
      parts[#parts + 1] = type(v) .. ":" .. tostring(v)
    end
  end
  return table.concat(parts, " | ")
end

local function try(p, method)
  if type(p[method]) ~= "function" then return end
  local res = { pcall(p[method]) }
  if res[1] then
    out(string.format("    %-24s -> %s", method .. "()", fmt(table.unpack(res, 2))))
  else
    out(string.format("    %-24s !! %s", method .. "()", tostring(res[2])))
  end
end

-- Методы по типам периферии (вытащены из классов Simulated 1.3.0)
local METHODS = {
  altitude_sensor   = { "getHeight", "getWorldHeight", "getAirPressure" },
  velocity_sensor   = { "getVelocity" },
  gimbal_sensor     = { "getAngles", "getAnglesRad" },
  navigation_table  = { "getRelativeAngle", "getRelativeAngleRad" },
  optical_sensor    = { "getDistance", "getBlock", "getRange" },
  directional_link  = { "getClosestAngle", "getClosestAngleRad" },
  modulating_link   = { "getClosestDistance" },
  swivel_bearing    = { "getTargetAngle", "getTargetAngleRad" },
  torsion_spring    = { "getAngle", "getAngleRad", "getLimit", "isRunning" },
  docking_connector = { "getConnectedName" },
  name_plate        = { "getName" },
  linked_typewriter = { "getPressedKeys", "getPressedKeyCodes" },
}

out("=== ПРОВЕРКА БОРТОВЫХ СИСТЕМ ===")
out(os.date("%Y-%m-%d %H:%M:%S"))
out("")

-- 1. Инвентаризация периферии
local names = peripheral.getNames()
out("Найдено периферии: " .. #names)
out("")

for _, name in ipairs(names) do
  local ptype = peripheral.getType(name)
  local p = peripheral.wrap(name)
  out(string.format("[%s]  тип=%s", name, tostring(ptype)))

  local known = METHODS[ptype]
  if known then
    for _, m in ipairs(known) do try(p, m) end
  else
    -- неизвестный тип — просто перечисляем что есть
    local ok, ms = pcall(peripheral.getMethods, name)
    if ok and ms then
      out("    методы: " .. table.concat(ms, ", "))
    end
  end
  out("")
end

-- 2. Работает ли GPS внутри суб-мира?
out("--- GPS ---")
if peripheral.find("modem") then
  local x, y, z = gps.locate(4)
  if x then
    out(string.format("gps.locate() -> %.2f, %.2f, %.2f", x, y, z))
    out("ВАЖНО: сверь эти цифры с F3 в игре.")
    out("Совпали  -> навигацию строим на GPS.")
    out("Разошлись-> это координаты суб-мира, переходим на маяки.")
  else
    out("gps.locate() не ответил (нет созвездия или вне радиуса).")
  end
else
  out("Модем не найден — поставь беспроводной модем на компьютер.")
end
out("")

-- 3. Динамика: 5 секунд телеметрии, чтобы увидеть шум и диапазоны
out("--- ТЕЛЕМЕТРИЯ, 5 с ---")
local alt = peripheral.find("altitude_sensor")
local vel = peripheral.find("velocity_sensor")
local gim = peripheral.find("gimbal_sensor")

if alt or vel or gim then
  for i = 1, 25 do
    local row = { string.format("t=%4.1f", i * 0.2) }
    if alt then row[#row + 1] = "alt=" .. fmt(alt.getHeight()) end
    if vel then row[#row + 1] = "vel=" .. fmt(vel.getVelocity()) end
    if gim then row[#row + 1] = "att=" .. fmt(gim.getAngles()) end
    out(table.concat(row, "  "))
    sleep(0.2)
  end
else
  out("Датчики движения не найдены.")
end

out("")
out("=== ГОТОВО, лог в " .. LOG .. " ===")
log.close()
