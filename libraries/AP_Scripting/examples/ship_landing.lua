-- This script implemented GUIDED mode ship landing for quadplanes

--mavlink system ID of ship beacon
local BEACON_SYS_ID = 17

-- height to hold above ship
local HOLD_HEIGHT = 50

-- flight mode numbers for plane
local MODE_GUIDED = 15

-- location, velocity and heading of target beacon
local target = Location()
local target_velocity = Vector3f()
local target_hdg_cd = 0

-- local time of last msg from beacon
local last_update_ms = 0

-- update target variables
function update_target()
   if not vehicle:set_follow_sysid(BEACON_SYS_ID) then
      return false
   end
   local ts1 = vehicle:get_follow_last_update_ms()
   target = vehicle:get_follow_location()
   target_velocity = vehicle:get_follow_velocity()
   target_hdg_cd = vehicle:get_follow_heading()
   ts2 = vehicle:get_follow_last_update_ms()
   if ts1:tofloat() == 0 or ts1:tofloat() ~= ts2:tofloat() then
      return false
   end
   last_update_ms = ts1
   return true
end

function update()
   -- only do anything if in GUIDED mode
   if vehicle:get_mode() ~= MODE_GUIDED then
      return update, 200
   end

   -- get updated target info
   if not update_target() then
      gcs:send_text(0, "no target")
      return update, 200
   end

   local home = ahrs:get_home()
   local current = ahrs:get_position()
   if not current then
      return update, 200
   end

   target:alt(home:alt() + HOLD_HEIGHT*100)
   vehicle:set_target_location(target)

   local dist = current:get_distance(target)
   gcs:send_text(0, "dist: " .. tostring(dist))

   -- run at 5Hz
   return update, 200
end

-- start running update loop
return update()
