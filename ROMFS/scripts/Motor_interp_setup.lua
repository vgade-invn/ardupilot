-- This script is an example setting up a custom interpolated motor matrix
-- this allows a 1D interpolation between roll pitch and yaw factors for up to 12 motors

-- 0 is the value this table will be used at, followed by a table per motor for its roll pitch and yaw factors
-- in the case of tilt rotors we use 1 for forward flight and 0 for hover

add_motors_interp_table(0.000000,
                       {{-0.500000, 0.500000, 0.500000},
                        {0.406086, -0.161268, 0.332390},
                        {0.500000, 0.500000, -0.500000},
                        {-0.406086, -0.161268, -0.332390}})

add_motors_interp_table(1.000000,
                        {{0.000000, 0.000000, 0.000000},
                         {0.000000, 0.000000, 0.000000},
                         {0.000000, 0.000000, 0.000000},
                         {0.000000, 0.000000, 0.000000}})

Motors_interp:add_motor(0, 1) -- motor number (zero indexed) and testing order (1 indexed)
Motors_interp:add_motor(1, 3)
Motors_interp:add_motor(2, 4)
Motors_interp:add_motor(3, 2)

-- try and init with 4 motors, each motor must have been added first
assert(Motors_interp:init(4), "Failed to init Motors_interp")

-- it is also possible to set the interpolation point from the script in real-time
Motors_interp:set_interpolation_point(0)
