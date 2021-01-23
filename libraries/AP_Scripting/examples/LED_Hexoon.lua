--[[
Script to control LED strips based on the roll of the aircraft. This is an example to demonstrate
the LED interface for WS2812 LEDs
--]]

--[[
for this demo we will use a single strip with 30 LEDs
--]]
local matrix_x = 7
local matrix_y = 7

-- matrix to convert from x y pos to location in the strip
local id = {}
-- because my strips go diagonally to get the led's closer together this is a odd ordering
id[1] = {}
id[1][1] = 21
id[1][2] = 20
id[1][3] = 10
id[1][4] = 9
id[1][5] = 3
id[1][6] = 2
id[1][7] = 0

id[2] = {}
id[2][1] = 33
id[2][2] = 22
id[2][3] = 19
id[2][4] = 11
id[2][5] = 8
id[2][6] = 4
id[2][7] = 1

id[3] = {}
id[3][1] = 34
id[3][2] = 32
id[3][3] = 23
id[3][4] = 18
id[3][5] = 12
id[3][6] = 7
id[3][7] = 5

id[4] = {}
id[4][1] = 42
id[4][2] = 35
id[4][3] = 31
id[4][4] = 24
id[4][5] = 17
id[4][6] = 13
id[4][7] = 6

id[5] = {}
id[5][1] = 43
id[5][2] = 41
id[5][3] = 36
id[5][4] = 30
id[5][5] = 25
id[5][6] = 16
id[5][7] = 14

id[6] = {}
id[6][1] = 47
id[6][2] = 44
id[6][3] = 40
id[6][4] = 37
id[6][5] = 29
id[6][6] = 26
id[6][7] = 15

id[7] = {}
id[7][1] = 48
id[7][2] = 46
id[7][3] = 45
id[7][4] = 39
id[7][5] = 38
id[7][6] = 28
id[7][7] = 27

-- https://github.com/noopkat/oled-font-5x7/blob/master/oled-font-5x7.js
font = {}
font[' '] = {0x00, 0x00, 0x00, 0x00, 0x00} -- // space
font['!'] = {0x00, 0x00, 0x5F, 0x00, 0x00} -- // !
font['"'] = {0x00, 0x07, 0x00, 0x07, 0x00} -- // "
font['#'] = {0x14, 0x7F, 0x14, 0x7F, 0x14} -- // #
font['$'] = {0x24, 0x2A, 0x7F, 0x2A, 0x12} -- // $
font['%'] = {0x23, 0x13, 0x08, 0x64, 0x62} -- // %
font['&'] = {0x36, 0x49, 0x55, 0x22, 0x50} -- // &
font['('] = {0x00, 0x1C, 0x22, 0x41, 0x00} -- // (
font[')'] = {0x00, 0x41, 0x22, 0x1C, 0x00} -- // )
font['*'] = {0x08, 0x2A, 0x1C, 0x2A, 0x08} -- // *
font['+'] = {0x08, 0x08, 0x3E, 0x08, 0x08} -- // +
font[','] = {0x00, 0x50, 0x30, 0x00, 0x00} -- // ,
font['-'] = {0x08, 0x08, 0x08, 0x08, 0x08} -- // -
font['.'] = {0x00, 0x60, 0x60, 0x00, 0x00} -- // .
font['/'] = {0x20, 0x10, 0x08, 0x04, 0x02} -- // /
font['0'] = {0x3E, 0x51, 0x49, 0x45, 0x3E} -- // 0
font['1'] = {0x00, 0x42, 0x7F, 0x40, 0x00} -- // 1
font['2'] = {0x42, 0x61, 0x51, 0x49, 0x46} -- // 2
font['3'] = {0x21, 0x41, 0x45, 0x4B, 0x31} -- // 3
font['4'] = {0x18, 0x14, 0x12, 0x7F, 0x10} -- // 4
font['5'] = {0x27, 0x45, 0x45, 0x45, 0x39} -- // 5
font['6'] = {0x3C, 0x4A, 0x49, 0x49, 0x30} -- // 6
font['7'] = {0x01, 0x71, 0x09, 0x05, 0x03} -- // 7
font['8'] = {0x36, 0x49, 0x49, 0x49, 0x36} -- // 8
font['9'] = {0x06, 0x49, 0x49, 0x29, 0x1E} -- // 9
font[':'] = {0x00, 0x36, 0x36, 0x00, 0x00} -- // :
font[';'] = {0x00, 0x56, 0x36, 0x00, 0x00} -- // ;
font['<'] = {0x00, 0x08, 0x14, 0x22, 0x41} -- // <
font['='] = {0x14, 0x14, 0x14, 0x14, 0x14} -- // =
font['>'] = {0x41, 0x22, 0x14, 0x08, 0x00} -- // >
font['?'] = {0x02, 0x01, 0x51, 0x09, 0x06} -- // ?
font['@'] = {0x32, 0x49, 0x79, 0x41, 0x3E} -- // @
font['A'] = {0x7E, 0x11, 0x11, 0x11, 0x7E} -- // A
font['B'] = {0x7F, 0x49, 0x49, 0x49, 0x36} -- // B
font['C'] = {0x3E, 0x41, 0x41, 0x41, 0x22} -- // C
font['D'] = {0x7F, 0x41, 0x41, 0x22, 0x1C} -- // D
font['E'] = {0x7F, 0x49, 0x49, 0x49, 0x41} -- // E
font['F'] = {0x7F, 0x09, 0x09, 0x01, 0x01} -- // F
font['G'] = {0x3E, 0x41, 0x41, 0x51, 0x32} -- // G
font['H'] = {0x7F, 0x08, 0x08, 0x08, 0x7F} -- // H
font['I'] = {0x00, 0x41, 0x7F, 0x41, 0x00} -- // I
font['J'] = {0x20, 0x40, 0x41, 0x3F, 0x01} -- // J
font['K'] = {0x7F, 0x08, 0x14, 0x22, 0x41} -- // K
font['L'] = {0x7F, 0x40, 0x40, 0x40, 0x40} -- // L
font['M'] = {0x7F, 0x02, 0x04, 0x02, 0x7F} -- // M
font['N'] = {0x7F, 0x04, 0x08, 0x10, 0x7F} -- // N
font['O'] = {0x3E, 0x41, 0x41, 0x41, 0x3E} -- // O
font['P'] = {0x7F, 0x09, 0x09, 0x09, 0x06} -- // P
font['Q'] = {0x3E, 0x41, 0x51, 0x21, 0x5E} -- // Q
font['R'] = {0x7F, 0x09, 0x19, 0x29, 0x46} -- // R
font['S'] = {0x46, 0x49, 0x49, 0x49, 0x31} -- // S
font['T'] = {0x01, 0x01, 0x7F, 0x01, 0x01} -- // T
font['U'] = {0x3F, 0x40, 0x40, 0x40, 0x3F} -- // U
font['V'] = {0x1F, 0x20, 0x40, 0x20, 0x1F} -- // V
font['W'] = {0x7F, 0x20, 0x18, 0x20, 0x7F} -- // W
font['X'] = {0x63, 0x14, 0x08, 0x14, 0x63} -- // X
font['Y'] = {0x03, 0x04, 0x78, 0x04, 0x03} -- // Y
font['Z'] = {0x61, 0x51, 0x49, 0x45, 0x43} -- // Z
font['['] = {0x00, 0x00, 0x7F, 0x41, 0x41} -- // [
font[']'] = {0x41, 0x41, 0x7F, 0x00, 0x00} -- // ]
font['^'] = {0x04, 0x02, 0x01, 0x02, 0x04} -- // ^
font['_'] = {0x40, 0x40, 0x40, 0x40, 0x40} -- // _
font['a'] = {0x20, 0x54, 0x54, 0x54, 0x78} -- // a
font['b'] = {0x7F, 0x48, 0x44, 0x44, 0x38} -- // b
font['c'] = {0x38, 0x44, 0x44, 0x44, 0x20} -- // c
font['d'] = {0x38, 0x44, 0x44, 0x48, 0x7F} -- // d
font['e'] = {0x38, 0x54, 0x54, 0x54, 0x18} -- // e
font['f'] = {0x08, 0x7E, 0x09, 0x01, 0x02} -- // f
font['g'] = {0x08, 0x14, 0x54, 0x54, 0x3C} -- // g
font['h'] = {0x7F, 0x08, 0x04, 0x04, 0x78} -- // h
font['i'] = {0x00, 0x44, 0x7D, 0x40, 0x00} -- // i
font['j'] = {0x20, 0x40, 0x44, 0x3D, 0x00} -- // j
font['k'] = {0x00, 0x7F, 0x10, 0x28, 0x44} -- // k
font['l'] = {0x00, 0x41, 0x7F, 0x40, 0x00} -- // l
font['m'] = {0x7C, 0x04, 0x18, 0x04, 0x78} -- // m
font['n'] = {0x7C, 0x08, 0x04, 0x04, 0x78} -- // n
font['o'] = {0x38, 0x44, 0x44, 0x44, 0x38} -- // o
font['p'] = {0x7C, 0x14, 0x14, 0x14, 0x08} -- // p
font['q'] = {0x08, 0x14, 0x14, 0x18, 0x7C} -- // q
font['r'] = {0x7C, 0x08, 0x04, 0x04, 0x08} -- // r
font['s'] = {0x48, 0x54, 0x54, 0x54, 0x20} -- // s
font['t'] = {0x04, 0x3F, 0x44, 0x40, 0x20} -- // t
font['u'] = {0x3C, 0x40, 0x40, 0x20, 0x7C} -- // u
font['v'] = {0x1C, 0x20, 0x40, 0x20, 0x1C} -- // v
font['w'] = {0x3C, 0x40, 0x30, 0x40, 0x3C} -- // w
font['x'] = {0x44, 0x28, 0x10, 0x28, 0x44} -- // x
font['y'] = {0x0C, 0x50, 0x50, 0x50, 0x3C} -- // y
font['z'] = {0x44, 0x64, 0x54, 0x4C, 0x44} -- // z
font['{'] = {0x00, 0x08, 0x36, 0x41, 0x00} -- // {
font['|'] = {0x00, 0x00, 0x7F, 0x00, 0x00} -- // |
font['}'] = {0x00, 0x41, 0x36, 0x08, 0x00} -- // }

--[[
 use SERVOn_FUNCTION 94 for LED. We can control up to 16 separate strips of LEDs
 by putting them on different channels
--]]
local chan_matrix = SRV_Channels:find_channel(94)
local chan_arm = SRV_Channels:find_channel(95)
local chan_arm_end = SRV_Channels:find_channel(96)

if not chan_matrix then
    gcs:send_text(6, "LEDs: channel 0 not set")
    return
end
if not chan_arm then
    gcs:send_text(6, "LEDs: channel 1 not set")
    return
end
if not chan_arm_end then
    gcs:send_text(6, "LEDs: channel 2 not set")
    return
end

-- find_channel returns 0 to 15, convert to 1 to 16
chan_matrix = chan_matrix + 1
chan_arm = chan_arm + 1
chan_arm_end = chan_arm_end + 1
gcs:send_text(6, "LEDs chan: " .. tostring(chan_matrix) .. ", " .. tostring(chan_arm)  .. ", " .. tostring(chan_arm_end))

local num_arms = 4
local num_leds = 6

-- RC input channels
local x_chan = 1
local y_chan = 2
local x_chan1 = 4
local y_chan1 = 3
local speed_chan = 11
local matrix_bright_chan = 12
local arm_bright_chan = 13
local string_switch_chan = 16
local arm_switch_chan = 15
local matrix_mode_chan = 9
local main_mode_chan = 10

-- initialization code
serialLED:set_num_profiled(chan_matrix,  matrix_x * matrix_y)
serialLED:set_num_profiled(chan_arm,  num_arms * num_leds * 2)
serialLED:set_num_neopixel(chan_arm_end,  num_arms * 7)

local offset = matrix_x
local text_string = "ArduPilot"
local display_string_length
local matrix_brightness
local arm_brightness
local speed
local tick = 0

local arm_offset = 0

local string_mode = 1
local reprint_mode = 10
local reprint_tick = 0

local ball_pos_x = 0
local ball_pos_y = 0
local ball_velo_x = 0
local ball_velo_y = 0
local play_state = 0
local score_left = 0
local score_right = 0

local function set_led_brightness(chan, index, r, g, b, brightness)
    r = math.ceil(r*brightness)
    g = math.ceil(g*brightness)
    b = math.ceil(b*brightness)

    serialLED:set_RGB(chan, index, r, g, b)
end

local function display_char(char,r,g,b,offset_in)
local char_offset = 0
if offset_in then
    char_offset = offset_in
end
if char_offset > matrix_x then
    return
end
if char_offset < 1 - 5 then
    return
end

local i
local j

for i = 1, 5 do
    local x_index = i + char_offset
    local font_colum = font[char][i]
    if font_colum ~= 0 then
        if x_index >= 1 and x_index <= matrix_x then
            for j = 1, 7 do
                if (font_colum & 1) == 1 then
                    set_led_brightness(chan_matrix, id[j][x_index], r, g, b,matrix_brightness)
                end
                font_colum = font_colum >> 1
            end
        end
    else
        char_offset = char_offset - 1
    end
end

end

-- some chars do not need the full 5 pixel width, for example "."
-- removing the extra white space makes words more readable
local function get_char_width(char_in)
    local width = 5
    for i = 1, 5 do
        if (font[char_in][i] == 0) then
            width = width - 1
        end
    end
    return width
end

local function get_display_length(text_string)
local length = 0
for i = 1, text_string:len() do
    length = length + get_char_width(text_string:sub(i,i)) + 1
end
return length - 1
end

local function display_string(string,r,g,b,offset_in)
local str_offset = 0
local i
for i = 1, string:len() do
    local string_char = string:sub(i,i)
    display_char(string_char,r,g,b,str_offset + offset_in)
    str_offset = str_offset + get_char_width(string_char) + 1
end
end

local function get_rc_scaled(chan)
    local pwm = 1500--rc:get_pwm(chan)
    local scaling = 0.05 -- default to low bigness for bench testing
    if pwm ~= 0 then
        -- assume default 1000 to 2000 range
        scaling = (pwm - 1000) / 1000
        -- constrain 0 to 1
        scaling = math.max(scaling,0)
        scaling = math.min(scaling,1)
    end
    return scaling
end

local function update_rc_speed()
    local pwm = 1500--rc:get_pwm(speed_chan)
    local speed_calc = 10
    if pwm ~= 0 then
        -- assume default 1000 to 2000 range
        speed_calc = ((pwm - 2000) / 1000) * -80
        -- constrain 1 to 80
        speed_calc = math.max(speed_calc,1)
        speed_calc = math.min(speed_calc,80)
    end
    speed = math.floor(speed_calc)
end

local function get_rc_mode(switch_chan)
    local mode = 0
    local pwm = 1500--rc:get_pwm(switch_chan)
    if pwm == 0 then 
        return 0
    end

    if pwm < 1200 then
        return 1
    elseif pwm > 1800 then
        return 3
    else
        return 2
    end
end

-- https://en.wikipedia.org/wiki/Gaussian_function
local function gaussian_1D(x, x_0, width)
    return math.exp( - ((x - x_0)^2) / width)
end
local function gaussian_2D(x, x_0, y, y_0, width)
    return math.exp( - ( (((x - x_0)^2) / width) + (((y - y_0)^2) / width) ))
end

local function display_arms()

local arm_mode = get_rc_mode(arm_switch_chan)
if arm_mode == 0 then
    arm_mode = 1
end

if arm_mode == 1 then
  -- running LED's, from inner to outer
  if arm_offset > num_leds then
    arm_offset = 0
  end

  local arm
  if arm_offset < num_leds then
    for arm = 0, num_arms - 1 do
        if arm < 2 then
            set_led_brightness(chan_arm, (arm * num_leds * 2) + arm_offset,255,0,0,arm_brightness)
            set_led_brightness(chan_arm, ((arm+1) * num_leds * 2) - arm_offset - 1, 255, 0, 0,arm_brightness)
        else
            set_led_brightness(chan_arm, (arm * num_leds * 2) + arm_offset, 0, 255, 0,arm_brightness)
            set_led_brightness(chan_arm, ((arm+1) * num_leds * 2) - arm_offset - 1, 0, 255, 0,arm_brightness)
        end
    end
  else
    for arm = 0, num_arms - 1 do
        local led_index
        for led_index = 0, 7 do
            if arm < 2 then
                set_led_brightness(chan_arm_end, (arm * 7) + led_index, 255, 0, 0,arm_brightness)
            else
                set_led_brightness(chan_arm_end, (arm * 7) + led_index, 0, 255, 0,arm_brightness)
            end
        end
    end
  end
end

if arm_mode == 2 then
  -- 'filling up' LED's, from inner to outer
  if arm_offset > num_leds+1 then
    arm_offset = 0
  end
  if arm_offset == num_leds+1 then
    return
  end
  local arm
  for arm = 0, num_arms - 1 do
    for arm_led = 0, math.min(arm_offset,num_leds-1) do
        if arm < 2 then
            set_led_brightness(chan_arm, (arm * num_leds * 2) + arm_led,255,0,0,arm_brightness)
            set_led_brightness(chan_arm, ((arm+1) * num_leds * 2) - arm_led - 1, 255, 0, 0,arm_brightness)
        else
            set_led_brightness(chan_arm, (arm * num_leds * 2) + arm_led, 0, 255, 0,arm_brightness)
            set_led_brightness(chan_arm, ((arm+1) * num_leds * 2) - arm_led - 1, 0, 255, 0,arm_brightness)
        end
    end
  end
  if arm_offset >= num_leds then
    for arm = 0, num_arms - 1 do
        local led_index
        for led_index = 0, 7 do
            if arm < 2 then
                set_led_brightness(chan_arm_end, (arm * 7) + led_index, 255, 0, 0,arm_brightness)
            else
                set_led_brightness(chan_arm_end, (arm * 7) + led_index, 0, 255, 0,arm_brightness)
            end
        end
    end
  end
end

if arm_mode == 3 then
    -- gaussian blob for smooth shift
    if arm_offset > num_leds then
      arm_offset = 0
    end

    local pos = speed * arm_offset + tick
    local width = 2 -- the width of the gaussian blob

    local arm
    for arm = 0, num_arms - 1 do
      for arm_led = 0, num_leds-1 do
          local gauss_scale = gaussian_1D(arm_led*speed,pos,width) * 255
          if arm < 2 then
              set_led_brightness(chan_arm, (arm * num_leds * 2) + arm_led,gauss_scale,0,0,arm_brightness)
              set_led_brightness(chan_arm, ((arm+1) * num_leds * 2) - arm_led - 1, gauss_scale, 0, 0,arm_brightness)
          else
              set_led_brightness(chan_arm, (arm * num_leds * 2) + arm_led, 0, gauss_scale, 0,arm_brightness)
              set_led_brightness(chan_arm, ((arm+1) * num_leds * 2) - arm_led - 1, 0, gauss_scale, 0,arm_brightness)
          end
      end
      local led_index
      gauss_scale = gaussian_1D(num_leds*speed,pos,width) * 255
      for led_index = 0, 7 do
          if arm < 2 then
              set_led_brightness(chan_arm_end, (arm * 7) + led_index, gauss_scale, 0, 0,arm_brightness)
          else
              set_led_brightness(chan_arm_end, (arm * 7) + led_index, 0, gauss_scale, 0,arm_brightness)
          end
      end
    end
  end

end

local function set_display_string(force)
    local mode = get_rc_mode(string_switch_chan)
    local string_changed = false
    if mode ~= string_mode then
      string_changed = true
      string_mode = mode
    end
  
   -- scroll until it is off the left edge
    if offset <= -display_string_length or string_changed or force then
      -- start with the stuff at the right edge of the display
      offset = matrix_x
  
      if string_changed or force then
          reprint_tick = 0
      end
  
      if string_mode == 1 then
          if reprint_tick == 0 then
              text_string = "Heading (deg)"
          else
              text_string = string.format("%.1f",math.deg(ahrs:get_yaw()))
          end
      end
  
      if string_mode == 2 then
          if reprint_tick == 0 then
              text_string = "Speed (m/s)"
          else
              local speed = ahrs:groundspeed_vector()
              text_string = string.format("%.1f",math.sqrt(speed:x()^2 + speed:y()^2))
          end
      end
  
      if string_mode == 3 then
          if reprint_tick == 0 then
              text_string = "Voltage (v)"
          else
              text_string = string.format("%.1f",battery:voltage(0))
          end
      end
  
      display_string_length = get_display_length(text_string)
      reprint_tick = reprint_tick + 1
      if reprint_tick >= reprint_mode then
          -- if no RC pick a random mode
          if string_mode == 0 then
              string_mode = math.random(3)
          end
           reprint_tick = 0
      end
    end
end

local function display_horizon()
local roll = ahrs:get_roll()
local pitch = ahrs:get_pitch()

-- this is the physical size of the matrix, so the angles look right
local matrix_dimension = 84.853 / matrix_x-- i used a jig (mm)
local matrix_distance = 73 -- the distance the matrix is in front of the CG (mm)

y_center = math.atan(pitch) * matrix_distance

x_change = math.atan(roll)

for x = 1, matrix_x do
    local pixel_x = matrix_dimension * (x - 4)
    local line_y = y_center - x_change * pixel_x
    for y = 1, matrix_y do
        local pixel_y = matrix_dimension * -(y - 4)
        local bright =  math.floor(255 * gaussian_1D(pixel_y,line_y,1500))
        if (x == 4 and  y == 4) or (x == 3 and y == 5) or (x == 5 and y == 5) then
            -- do red arrow
            set_led_brightness(chan_matrix, id[y][x], 255, 0, 0, matrix_brightness) 
        elseif pixel_y > line_y then
            --set_led_brightness(chan_matrix, id[y][x], 0, 0, 255, matrix_brightness)
            set_led_brightness(chan_matrix, id[y][x], 255 - bright, 255 - bright, 255, matrix_brightness)
        else
            --set_led_brightness(chan_matrix, id[y][x], 0, 255, 0, matrix_brightness)
            set_led_brightness(chan_matrix, id[y][x], 0, bright, 0, matrix_brightness)
        end
    end
end

end

local function display_sticks()
    local x_pwm = 1500--rc:get_pwm(x_chan)
    local y_pwm = 1500--rc:get_pwm(y_chan)
    local x_pwm1 = 1500--rc:get_pwm(x_chan1)
    local y_pwm1 = 1500--rc:get_pwm(y_chan1)

    -- convert to 1 to 7 range
    local x_loc = (((x_pwm - 1000) / 1000) * 6) + 1
    local y_loc = (((y_pwm - 1000) / 1000) * 6) + 1
    local x_loc1 = (((x_pwm1 - 1000) / 1000) * 6) + 1
    local y_loc1 = (((y_pwm1 - 2000) / -1000) * 6) + 1

    for x = 1, matrix_x do
        for y = 1, matrix_y do
            set_led_brightness(chan_matrix, id[y][x], 0, math.floor(255 * gaussian_2D(x,x_loc1,y,y_loc1,1)), math.floor(255 * gaussian_2D(x,x_loc,y,y_loc,1)), matrix_brightness)
        end
    end

end

local function display_pong()
    -- play_state
    -- 0 = init pong string
    -- 1 = display pong string
    -- 2 = init game, left first
    -- 3 = init game, right first
    -- 4 = play point
    -- 5 = left scores a point
    -- 6 = right scores a point

    if play_state == 0 then
        offset = matrix_x
        play_state = 1
        text_string = 'Pong'
        display_string_length = get_display_length(text_string)
        display_string(text_string,255,0,0,offset)
        score_left = 0
        score_right = 0
        return
    end
    if play_state == 1 then
        display_string(text_string,255,0,0,offset)
        if offset <= -display_string_length then
            -- randomly pick who goes first
            if math.random() > 0.5 then
                play_state = 2
            else
                play_state = 3
            end
        end
        return
    end

    local left_pwm = 1500--rc:get_pwm(y_chan1)
    local right_pwm = 1500--rc:get_pwm(y_chan)

    local left_loc = (((left_pwm - 2000) / -1000) * 6) + 1
    local right_loc = (((right_pwm - 1000) / 1000) * 6) + 1

    -- work out the ball positron
    if play_state == 2 or play_state == 3 then
        -- start at a random position on the center line
        ball_pos_x = 7/2
        ball_pos_y = math.random() * 7

        ball_velo_x = (math.random() * 0.5) + 0.75
        if play_state == 2 then
            ball_velo_x = ball_velo_x * -1
        end
        ball_velo_y = (math.random() - 0.5) * 2
        play_state = 4
        return
    end

    if play_state == 4 then
        local ball_speed = ((80 - speed) / 1500)
        ball_pos_x = ball_pos_x + ball_velo_x * ball_speed
        ball_pos_y = ball_pos_y + ball_velo_y * ball_speed

        if ball_pos_x > 7 then
            ball_pos_x = 7
            ball_velo_x = -ball_velo_x
            if gaussian_1D(ball_pos_y,right_loc,0.25) < 0.15 then
                -- missed
                score_left = score_left + 1
                play_state = 5
                offset = 6
                notify:play_tune('MF L2 B')
            else 
                notify:play_tune('MF L16 B')
            end
        end
        if ball_pos_y > 7 then
            ball_pos_y = 7
            ball_velo_y = -ball_velo_y
            notify:play_tune('MF L16 C-')
        end
        if ball_pos_x < 1 then
            ball_pos_x = 1
            ball_velo_x = -ball_velo_x
            if gaussian_1D(ball_pos_y,left_loc,0.25) < 0.15 then
                -- missed
                score_right = score_right + 1
                play_state = 6
                offset = 6
                notify:play_tune('MF L2 B')
            else 
                notify:play_tune('MF L16 B')
            end
        end
        if ball_pos_y < 1 then
            ball_pos_y = 1
            ball_velo_y = -ball_velo_y
            notify:play_tune('MF L16 C-')
        end
    end

    if score_right > 1 then
        for arm_index = 0, score_right - 2 do
            set_led_brightness(chan_arm, arm_index,0,255,0,arm_brightness)
        end
    end
    if score_left > 1 then
        for arm_index = 0, score_left - 2 do
            set_led_brightness(chan_arm, ((6*8)-arm_index) - 1,0,0,255,arm_brightness)
        end
    end
    local display_ball = true
    if play_state == 5 or play_state == 6 then
        if  offset % 2 == 0 then
            display_ball = false
        end
        if offset == 0 then
            if score_left == 6 then
                offset = matrix_x
                text_string = 'Blue Wins!'
                display_string_length = get_display_length(text_string)
                display_string(text_string,255,0,0,offset)
                score_left = 0
                score_right = 0
                play_state = 1
                return
            end
            if score_right == 6 then
                offset = matrix_x
                text_string = 'Green Wins!'
                display_string_length = get_display_length(text_string)
                display_string(text_string,255,0,0,offset)
                score_left = 0
                score_right = 0
                play_state = 1
                return
            end
            if play_state == 5 then
                play_state = 2
            else
                play_state = 3
            end
        end
    end
    if display_ball then
        if score_right > 0 then
            set_led_brightness(chan_arm, score_right - 1,0,255,0,arm_brightness)
        end
        set_led_brightness(chan_arm, ((6*8)-score_left),0,0,255,arm_brightness)
    else
        if play_state == 5 and score_right > 0 then
            set_led_brightness(chan_arm, score_right-1,0,255,0,arm_brightness)
        end
        if play_state == 6 then
            set_led_brightness(chan_arm, ((6*8)-score_left),0,0,255,arm_brightness)
        end
    end

    for x = 1, matrix_x do
        for y = 1, matrix_y do
            if display_ball then
                if x == 1 then
                    set_led_brightness(chan_matrix, id[y][x], math.floor(255 * gaussian_2D(x,ball_pos_x,y,ball_pos_y,0.25)), 0, math.floor(255 * gaussian_1D(y,left_loc,0.25)), matrix_brightness)
                elseif x == 7 then
                    set_led_brightness(chan_matrix, id[y][x], math.floor(255 * gaussian_2D(x,ball_pos_x,y,ball_pos_y,0.25)), math.floor(255 * gaussian_1D(y,right_loc,0.25)), 0 , matrix_brightness)
                elseif x == 4 then
                    set_led_brightness(chan_matrix, id[y][x], math.floor(255 * gaussian_2D(x,ball_pos_x,y,ball_pos_y,0.25)) + 255 * 0.01, 255 * 0.01, 255 * 0.01, matrix_brightness)
                else
                    set_led_brightness(chan_matrix, id[y][x], math.floor(255 * gaussian_2D(x,ball_pos_x,y,ball_pos_y,0.25)), 0, 0, matrix_brightness)
                end
            else
                if x == 1 then
                    set_led_brightness(chan_matrix, id[y][x], 0, 0, math.floor(255 * gaussian_1D(y,left_loc,0.25)), matrix_brightness)
                elseif x == 7 then
                    set_led_brightness(chan_matrix, id[y][x], 0, math.floor(255 * gaussian_1D(y,right_loc,0.25)), 0 , matrix_brightness)
                elseif x == 4 then
                    set_led_brightness(chan_matrix, id[y][x], 255 * 0.01, 255 * 0.01, 255 * 0.01, matrix_brightness)
                else
                    set_led_brightness(chan_matrix, id[y][x], 0, 0, 0, matrix_brightness)
                end
            end
        end
    end
end

function update_LEDs()

  serialLED:set_RGB(chan_matrix, -1, 0, 0, 0)
  serialLED:set_RGB(chan_arm, -1, 0, 0, 0)
  serialLED:set_RGB(chan_arm_end, -1, 0, 0, 0)

  matrix_brightness = get_rc_scaled(matrix_bright_chan)
  arm_brightness = get_rc_scaled(arm_bright_chan)
  update_rc_speed()

  local main_mode = get_rc_mode(main_mode_chan)
  if main_mode ~= 3 and play_state <= 1 then
    if play_state ~= 0 then
        play_state = 0
        set_display_string(true)
    end
    local matrix_mode = get_rc_mode(matrix_mode_chan)
    if matrix_mode == 1 or matrix_mode == 0 then
        display_string(text_string,0,0,255,offset)
        set_display_string()
    end
    if matrix_mode == 2 then
        display_horizon()
        offset = matrix_x -- reset the string display
    end
    if matrix_mode == 3 then
        display_sticks()
        offset = matrix_x
    end
    
    display_arms()
  else
    display_pong()
  end

  serialLED:send(chan_arm)
  serialLED:send(chan_arm_end)

  tick = tick + 1
  if tick > speed then
    offset = offset - 1

    arm_offset = arm_offset + 1

    tick = 0
  end

  return update_LEDs, 10
end

-- work out the pixel width of the string to be displayed
display_string_length = get_display_length(text_string)

return update_LEDs, 1000

