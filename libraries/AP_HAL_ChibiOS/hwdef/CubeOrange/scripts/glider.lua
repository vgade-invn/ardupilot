local MODE_AUTO = 10

local FEET_TO_METERS = 0.3048
local METERS_TO_FEET = 1.0/FEET_TO_METERS

local KNOTS_TO_MPS = 0.51444

-- altitude to force chute open if in AUTO and we've cut balloon free
local CHUTE_OPEN_ALT = 2700*FEET_TO_METERS

local K_PARACHUTE = 27

local NAVLIGHTS_CHAN = 8
local BALLOON_RELEASE_CHAN = 10

-- chute checking is enabled when 50m above chute deploy alt
local chute_check_armed = false
local chute_check_margin = 50
local target_keas = 55

-- constrain a value between limits
function constrain(v, vmin, vmax)
   if v < vmin then
      v = vmin
   end
   if v > vmax then
      v = vmax
   end
   return v
end

--[[
table of fence limits from home by alt AMSL
altitudes in feet, limits in meters
--]]
local fence_limits = {
  { 9000, 60000 }, -- 30km at 9k ft
  { 6000, 20000 }, -- 20km at 6k ft
  { 3000, 14000 }, -- 14km at 3k ft
}

local fence_triggered = false
local chute_triggered = false

function get_dist_home()
   local loc = ahrs:get_position()
   local home = ahrs:get_home()
   if not loc or not home then
      -- no position or home yet, can't do fence
      return 0
   end
   return loc:get_distance(home)
end

function trigger_fence()
   if not fence_triggered then
      fence_triggered = true
      local loc = ahrs:get_position()
      local alt_amsl_ft = loc:alt() * 0.01 * METERS_TO_FEET
      parachute:release()
      gcs:send_text(0, string.format("Triggering fence limit at %.0fm alt %.0fft", get_dist_home(), alt_amsl_ft))
   end
end

function balloon_has_released()
   if SRV_Channels:get_output_pwm_chan(BALLOON_RELEASE_CHAN-1) >= 1750 then
      return true
   end
   return false
end


function check_fence()
   local loc = ahrs:get_position()
   if not loc then
      -- no position yet, can't do fence
      return
   end
   if not balloon_has_released() then
      -- balloon not released yet
      return
   end
   local dist_home = get_dist_home()
   local alt_amsl_ft = loc:alt() * 0.01 * METERS_TO_FEET
   local num_rows = #fence_limits

   for row = 1, num_rows do
      if alt_amsl_ft >= fence_limits[row][1] then
         if dist_home > fence_limits[row][2] then
            trigger_fence()
         end
         return
      end
   end

   -- we are below lowest level
   if dist_home > fence_limits[num_rows][2] then
      trigger_fence()
   end
end

--[[
KEAS schedule for 9k drop
--]]
local speeds_9k = {
  { 7000, 65 },
  { 5000, 70 },
  { 4000, 55 },
  { 3000, 35 },
}

--[[
KEAS schedule for 25k drop
--]]
local speeds_25k = {
  { 15000, 65 },
  { 10000, 70 },
  {  4000, 55 },
  {  3000, 35 },
}

--[[
KEAS schedule for 45k drop
--]]
local speeds_45k = {
  { 40000, 65 },
  { 35000, 70 },
  { 17000, 55 },
  {  3000, 35 },
}

--[[
KEAS schedule for 60k drop
--]]
local speeds_60k = {
  { 55000, 65 },
  { 45000, 70 },
  { 17000, 55 },
  {  3000, 35 },
}

--[[
KEAS schedule for 90k drop
--]]
local speeds_90k = {
  { 79000, 65 },
  { 45000, 70 },
  { 17000, 55 },
  {  3000, 35 },
}

function select_speed_schedule()
   kft = param:get('SCR_USER4')
   if kft == 9 then
      return speeds_90k
   end
   if kft == 25 then
      return speeds_25k
   end
   if kft == 45 then
      return speeds_45k
   end
   if kft == 60 then
      return speeds_60k
   end
   if kft == 90 then
      return speeds_90k
   end
   return nil
end

function adjust_target_speed()
   if not chute_check_armed then
      return
   end
   if not balloon_has_released() then
      -- balloon not released yet
      return
   end
   speed_schedule = select_speed_schedule()
   if not speed_schedule then
      return
   end

   local loc = ahrs:get_position()
   local alt_ft = loc:alt() * 0.01 * METERS_TO_FEET

   local num_rows = #speed_schedule

   target_speed_keas = 55
   for row = 1, num_rows do
      if alt_ft < speed_schedule[row][1] then
         target_speed_keas = speed_schedule[row][2]
      end
   end
   if target_speed_keas ~= target_keas then
      target_keas = target_speed_keas
      target_speed_cms = target_speed_keas * KNOTS_TO_MPS * 100.0
      current_cms = param:get('TRIM_ARSPD_CM')
      if not current_cms or math.abs(current_cms - target_speed_cms) > 5.0 then
         gcs:send_text(0, string.format("Target speed %.1f KEAS at %.0fft", target_keas, alt_ft))
         param:set('TRIM_ARSPD_CM', target_speed_cms)
      end
   end
end

function check_chute()
   local loc = ahrs:get_position()
   local alt = loc:alt() * 0.01
   if not chute_check_armed then
      if alt > CHUTE_OPEN_ALT + chute_check_margin then
         gcs:send_text(0, string.format("Armed chute check at %.0fft", alt*METERS_TO_FEET))
         chute_check_armed = true
      end
   end
   if chute_check_armed and alt < CHUTE_OPEN_ALT then
      if not chute_triggered then
         chute_triggered = true
         gcs:send_text(0, string.format("Triggering chute at %.0fft", alt*METERS_TO_FEET))
         parachute:release()
      end
   end
end

function update_lights()
   if arming:is_armed() then
      SRV_Channels:set_output_pwm_chan(NAVLIGHTS_CHAN-1, 1400)
   else
      SRV_Channels:set_output_pwm_chan(NAVLIGHTS_CHAN-1, 1000)
   end
end

function update()
   update_lights()
   if arming:is_armed() and vehicle:get_mode() == MODE_AUTO then
      check_fence()
      check_chute()
      adjust_target_speed()
   end

   return update, 1000
end

gcs:send_text(0, string.format("Loader glider script"))

return update, 1000
