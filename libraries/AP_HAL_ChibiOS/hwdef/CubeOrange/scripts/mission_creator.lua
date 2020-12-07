-- auto-create drop mission

DO_JUMP = 177
NAV_WAYPOINT = 16
NAV_LAND = 21
DO_LAND_START = 189

APPROACH_DIST_MAX = 6400
BASE_DIST = 1390
STEP_RATIO = 0.5

local GLIDE_SLOPE = 6.0

-- see if mission is setup for auto-creation
function create_mission_check()
   if mission:num_commands() ~= 4 then
      return false
   end
   if mission:get_item(1):command() ~= NAV_LAND or mission:get_item(2):command() ~= NAV_LAND then
      return false
   end
   if mission:get_item(3):command() ~= DO_LAND_START then
      return false
   end
   return true
end

function get_glide_slope()
   GLIDE_SLOPE = param:get('SCR_USER2')
   if GLIDE_SLOPE <= 0 then
      GLIDE_SLOPE = 6.0
   end
end

-- set wp location
function wp_setloc(wp, loc)
   wp:x(loc:lat())
   wp:y(loc:lng())
   wp:z(loc:alt()*0.01)
end

-- get wp location
function wp_getloc(wp)
   local loc = Location()
   loc:lat(wp:x())
   loc:lng(wp:y())
   loc:alt(math.floor(wp:z()*100))
   return loc
end

-- shift a wp in polar coordinates
function wp_offset(wp, ang1, dist1)
   local loc = wp_getloc(wp)
   loc:offset_bearing(ang1,dist1)
   wp:x(loc:lat())
   wp:y(loc:lng())
end

-- copy and shift a location
function loc_copy(loc,angle,dist)
   local loc2 = Location()
   loc2:lat(loc:lat())
   loc2:lng(loc:lng())
   loc2:alt(loc:alt())
   loc2:offset_bearing(angle,dist)
   return loc2
end

-- adjust approach distance based on glide slope and location of LAND1
-- and DO_LAND_START
function adjust_approach_dist(land1_loc, dls_loc, angle)
   local alt_change = (dls_loc:alt() - land1_loc:alt()) * 0.01
   local target_dist = 1.3 * GLIDE_SLOPE * alt_change
   gcs:send_text(0, string.format("alt_change=%.2f target_dist=%.2f", alt_change, target_dist))

   -- converge on correct approach distance
   local dist
   for i = 1, 10 do
      local loc2 = loc_copy(land1_loc, angle, APPROACH_DIST_MAX)
      dist = land1_loc:get_distance(loc2) + loc2:get_distance(dls_loc)
      APPROACH_DIST_MAX = APPROACH_DIST_MAX * target_dist / dist
   end
   gcs:send_text(0, string.format("app_dist=%.2f dist=%.2f target_dist=%.2f", APPROACH_DIST_MAX, dist, target_dist))
end

function get_location(i)
   local m = mission:get_item(i)
   local loc = Location()
   loc:lat(m:x())
   loc:lng(m:y())
   loc:relative_alt(false)
   loc:terrain_alt(false)
   loc:origin_alt(false)
   loc:alt(math.floor(m:z()*100))
   return loc
end

function create_pattern(wp, basepos, angle, base_angle, runway_length)

   local jump_target = mission:num_commands() + 3

   wp:command(NAV_WAYPOINT)
   wp_setloc(wp, basepos)
   wp_offset(wp, angle, APPROACH_DIST_MAX)
   wp_offset(wp, base_angle, BASE_DIST)
   mission:set_item(mission:num_commands(), wp)

   wp_setloc(wp, basepos)
   wp_offset(wp, angle, APPROACH_DIST_MAX)
   mission:set_item(mission:num_commands(), wp)

   wp_setloc(wp, basepos)
   wp_offset(wp, angle, runway_length*1.2)
   mission:set_item(mission:num_commands(), wp)

   wp_setloc(wp, basepos)
   wp:command(NAV_LAND)
   mission:set_item(mission:num_commands(), wp)

   local step = runway_length*STEP_RATIO
   NUM_ALTERNATES = math.min(70,math.floor((APPROACH_DIST_MAX - runway_length*1.5) / step))

   gcs:send_text(0, string.format("NUM_ALTERNATES=%u", NUM_ALTERNATES))

   for i = 1, NUM_ALTERNATES do
      wp:command(DO_JUMP)
      wp:param1(jump_target)
      wp:param2(-1)
      mission:set_item(mission:num_commands(), wp)

      wp_setloc(wp, basepos)
      wp:command(NAV_WAYPOINT)
      wp_offset(wp, angle, APPROACH_DIST_MAX - step*i)
      wp_offset(wp, base_angle, BASE_DIST)
      mission:set_item(mission:num_commands(), wp)

      wp_setloc(wp, basepos)
      wp_offset(wp, angle, APPROACH_DIST_MAX - step*i)
      mission:set_item(mission:num_commands(), wp)
   end

   wp:command(DO_JUMP)
   wp:param1(jump_target)
   wp:param2(-1)
   mission:set_item(mission:num_commands(), wp)
end

-- auto-create mission
function create_mission()
   gcs:send_text(0, string.format("creating mission"))
   local land1_loc = get_location(1)
   local land2_loc = get_location(2)
   local dls = mission:get_item(3)
   local dls_loc = get_location(3)
   local runway_length = land1_loc:get_distance(land2_loc)

   local alt_agl = (dls_loc:alt() - land1_loc:alt()) * 0.01
   if alt_agl <= 0 then
      gcs:send_text(0, string.format("invalid altitudes"))
      return
   end

   -- refresh glide slope
   get_glide_slope()

   local flight_dist1 = dls_loc:get_distance(land1_loc)
   local flight_dist2 = dls_loc:get_distance(land2_loc)
   local slope1 = flight_dist1 / alt_agl
   local slope2 = flight_dist2 / alt_agl

   if slope1 > GLIDE_SLOPE then
      gcs:send_text(0, string.format("bad glide slope1 %.2f", slope1))
      return
   end
   if slope2 > GLIDE_SLOPE then
      gcs:send_text(0, string.format("bad glide slope2 %.2f", slope2))
      return
   end
   

   local bearing12 = math.deg(land1_loc:get_bearing(land2_loc))
   local bearing21 = math.deg(land2_loc:get_bearing(land1_loc))
   local base_angle1 = bearing12 + 90
   local base_angle2 = bearing12 - 90

   -- work out which base angle brings us closer to the DLS
   local base_dist1 = loc_copy(land1_loc,base_angle1,10):get_distance(dls_loc)
   local base_dist2 = loc_copy(land1_loc,base_angle2,10):get_distance(dls_loc)
   if base_dist1 < base_dist2 then
      base_angle = base_angle1
   else
      base_angle = base_angle2
   end

   adjust_approach_dist(land1_loc, dls_loc, bearing12)

   local wp = mission:get_item(1)
   wp:command(NAV_WAYPOINT)

   mission:clear()

   -- setup home
   wp_setloc(wp, dls_loc)
   mission:set_item(mission:num_commands(), wp)

   -- setup DLS1
   mission:set_item(mission:num_commands(), dls)

   -- first pattern
   create_pattern(wp, land1_loc, bearing12, base_angle, runway_length)

   -- setup DLS2
   mission:set_item(mission:num_commands(), dls)

   -- second pattern
   create_pattern(wp, land2_loc, bearing21, base_angle, runway_length)

end

function update()
   if rc:has_valid_input() and rc:get_pwm(8) > 1800 then
      -- disable automation
      return update, 1000
   end

   if not arming:is_armed() and create_mission_check() then
      create_mission()
   end

   return update, 2000
end

return update() -- run immediately before starting to reschedule
