-- This script implemented GUIDED mode ship landing for quadplanes

--mavlink system ID of ship beacon
local BEACON_SYS_ID = 17

-- height to hold above ship
local HOLD_HEIGHT = 50

-- flight mode numbers for plane
local MODE_GUIDED = 15
local MODE_RTL = 11
local MODE_QRTL = 21

-- standoff distance for loiter
local STANDOFF_DIST = 150

-- location, velocity and heading of target beacon
local target = Location()
local target_velocity = Vector3f()
local target_hdg_cd = 0

-- local time of last msg from beacon
local last_update_ms = 0

local UINT16_MAX = 65535

function radians(x)
   return x * 0.0174532925199
end

-- update target variables
function update_target()
   if not vehicle:set_follow_sysid(BEACON_SYS_ID) then
      return false
   end
   -- we loop until we get the same timestamp twice, avoiding a race condition in fetch data
   -- from multiple GLOBAL_POSITION_INT packets
   local ts1, ts2
   repeat
      ts1 = vehicle:get_follow_last_update_ms()
      target = vehicle:get_follow_location()
      target_velocity = vehicle:get_follow_velocity()
      target_hdg_cd = vehicle:get_follow_heading()
      ts2 = vehicle:get_follow_last_update_ms()
   until ts1 == ts2
   if ts1 == uint32_t(0) then
      return false
   end
   last_update_ms = ts1
   return true
end

function update()
   -- get updated target info
   if not update_target() then
      gcs:send_text(0, "no target")
      return update, 200
   end

   local current = ahrs:get_position()
   if not current then
      return update, 200
   end

   -- advance target by time delta to account for comms lag
   local dt = (millis() - last_update_ms):tofloat() * 0.001
   local dpos = target_velocity:scale(dt)
   target:offset(dpos:x(), dpos:y())

   -- set home to ship
   local mode = vehicle:get_mode()
   if mode == MODE_GUIDED or mode == MODE_RTL or mode == MODE_QRTL then
      vehicle:set_home(target)
   end

   -- only do the rest if in GUIDED mode
   if mode ~= MODE_GUIDED then
      return update, 200
   end

   -- hold at HOLD_HEIGHT above target
   target:alt(target:alt() + HOLD_HEIGHT*100)

   -- hold to rear and right of target by STANDOFF_DIST
   if target_hdg_cd ~= UINT16_MAX then
      hdg_rad = radians(target_hdg_cd*0.01)
      offset = Vector2f()
      offset:x(-STANDOFF_DIST)
      offset:y(STANDOFF_DIST)
      offset:rotate(hdg_rad)
      target:offset(offset:x(), offset:y())
   end

   vehicle:set_target_location(target)

   local dist = current:get_distance(target)
   gcs:send_text(0, "dist: " .. tostring(dist))

   -- run at 5Hz
   return update, 200
end

-- start running update loop
return update()
