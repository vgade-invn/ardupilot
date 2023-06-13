

local SEVERITY_EMERGENCY  = '0' -- # Emergency: System is unusable. This is a "panic" condition.
local SEVERITY_ALERT      = '1' -- # Alert: Action should be taken immediately. Indicates error in non-critical systems.
local SEVERITY_CRITICAL   = '2' -- # Critical: Action must be taken immediately. Indicates failure in a primary system.
local SEVERITY_ERROR      = '3' -- # Error: Indicates an error in secondary/redundant systems.
local SEVERITY_WARNING    = '4' -- # Warning: Indicates about a possible future error if this is not resolved within a given timeframe. Example would be a low battery warning.
local SEVERITY_NOTICE     = '5' -- # Notice: An unusual event has occurred, though not an error condition. This should be investigated for the root cause.
local SEVERITY_INFO       = '6' -- # Info: Normal operational messages. Useful for logging. No action is required for these messages.
local SEVERITY_DEBUG      = '7' -- # Debug: Useful non-operational messages that can assist in debugging. These should not occur during normal operation.


-- config definitions
UPDATE_RATE         = 5000
MAX_ESC             = 4

-- CONSTS
EFI_WARMUP_TEMP1    = 120
EFI_WARMUP_TEMP2    = 120
EFI_CRITICAL_RPM    = 2000

ESC_TIMEOUT         = 2000

-- check_state
Prearm_check_state = {
    DISARMED_CHECK_ONCE         = 1,
    DISARMED_CHECK_CONTINUOUS   = 2,
    ARMED_CHECK_ONCE            = 3,
    ARMED_CHECK_CONTINUOUS      = 4,
    UNDEFINED_CHECKS            = 99,
    STARTUP_CHECK               = 999,
}

Check_efi_state = Prearm_check_state.STARTUP_CHECK
Check_esc_state = Prearm_check_state.STARTUP_CHECK


-- set ESC data table
Esc_type = {
    index = 0,
    prev_ts = 0,
    ts = 0,
    ts_flag = false,
    temp = 0,
    temp_flag = false,
    rpm = 0,
    prev_rpm = 0
}

local esc_type1 = Esc_type
local esc_type2 = Esc_type
local esc_type3 = Esc_type
local esc_type4 = Esc_type

Esc_list = { esc_type1, esc_type2, esc_type3, esc_type4 }
local esc_list = Esc_list

RC_out_list = {33, 34, 35, 36}

Prev_rcout = {}
Curr_rcout = {}
Prev_esc_rpm = {}



-- Check if lua configuration settings are proper
function Check_lua()
    -- if 0 then
    --    gcs:send_text(SEVERITY_CRITICAL, string.format( "LUA not init"))
    -- end
end


-- set initial state for pre-arm check conditions
function Init_check_status()

    for index=1, MAX_ESC, 1 do
        -- esc_list[index].prev_ts = esc_telem.get_last_telem_data_ms(index)
        -- local temp = esc_telem.get_last_telem_data_ms(index, temp1)
        -- gcs:send_text(SEVERITY_CRITICAL, string.format("ESC:%d delta:%d", index+1, temp1))
        esc_list[index].temp_flag = false
        Prev_rcout[index] = 1
    end

    -- make sure ESC timeout error is incorrectly printed
    if ESC_TIMEOUT < UPDATE_RATE then
        ESC_TIMEOUT = UPDATE_RATE
    end

    Check_efi_state = Prearm_check_state.DISARMED_CHECK_ONCE
    Check_esc_state = Prearm_check_state.DISARMED_CHECK_ONCE
end


-- main function call to check EFI functionalities
function Check_efi_status()
    if Check_efi_state == Prearm_check_state.DISARMED_CHECK_ONCE then
        if arming:is_armed() then
            Check_efi_state = Prearm_check_state.ARMED_CHECK_ONCE
        end
    elseif Check_efi_state == Prearm_check_state.DISARMED_CHECK_CONTINUOUS then
        if arming:is_armed() then
            Check_efi_state = Prearm_check_state.ARMED_CHECK_ONCE
        end
    elseif Check_efi_state == Prearm_check_state.ARMED_CHECK_ONCE then
        if not arming:is_armed() then
            Check_efi_state = Prearm_check_state.DISARMED_CHECK_ONCE
        end
        Check_efi_status_once()
    elseif Check_efi_state == Prearm_check_state.ARMED_CHECK_CONTINUOUS then
        if not arming:is_armed() then
            Check_efi_state = Prearm_check_state.DISARMED_CHECK_ONCE
        end
        Check_efi_status_continuous()
    else
        return Protected_wrapper, UPDATE_RATE
    end
end


-- Check EFI status checks until engine temperature is warmed up
function Check_efi_status_once()
    -- if not reading rpm, discontinue check
    local rpm = efi:get_rpm()
    if rpm == nil then
        gcs:send_text(SEVERITY_CRITICAL, string.format("Engine Not Detected"))
        return
    end

    -- ensure RPM is not too low
    if rpm < EFI_CRITICAL_RPM then
        gcs:send_text(SEVERITY_ERROR, string.format("Engine RPM Too LOW"))
    end

    -- check until temperature > WARMUP_TEMP 
    local cht1_temp, cht2_temp, egt1_temp, egt2_temp, air_temp, eng_temp = efi:get_temp()
    if (cht1_temp >= EFI_WARMUP_TEMP1) and (cht2_temp >= EFI_WARMUP_TEMP2) then
        gcs:send_text(SEVERITY_INFO, string.format("Engine Warmed UP"))
        Check_efi_state = Prearm_check_state.ARMED_CHECK_CONTINUOUS
    end

end


-- Check EFI status
function Check_efi_status_continuous()
    -- if not reading rpm, discontinue check
    local rpm = efi:get_rpm()
    if rpm == nil then
        gcs:send_text(SEVERITY_CRITICAL, string.format("Engine Not Detected"))
        return
    end

    -- ensure RPM is not too low
    if rpm < EFI_CRITICAL_RPM then
        gcs:send_text(SEVERITY_ERROR, string.format("Engine RPM Too LOW"))
    end

    -- check until temperature > WARMUP_TEMP 
    local cht1_temp, cht2_temp, egt1_temp, egt2_temp, air_temp, eng_temp = efi:get_temp()
    if (cht1_temp <= EFI_WARMUP_TEMP1 + 2) and (cht2_temp <= EFI_WARMUP_TEMP2 + 2) then
        gcs:send_text(SEVERITY_INFO, string.format("Engine COOLED"))
        Check_efi_state = Prearm_check_state.ARMED_CHECK_ONCE
    end
end


-- main function for esc testing
function Check_esc_status()
    if arming:is_armed() then
        Check_esc_status_continuous()
    end
end


-- Check ESC status
function Check_esc_status_once()
    local status = 0

    for index = 1, MAX_ESC, 1
    do
        esc_list[index].ts = esc_telem.get_last_telem_data_ms(index)
        if (esc_list[index].ts ~= nil) then
            status = status + 1
        else
            gcs:send_text(SEVERITY_INFO, string.format("ESC" .. index .."Not working"))
        end
    end
    if (status == 4) then
        Check_esc_status_continuous()
        gcs:send_text(SEVERITY_INFO, string.format("All ESCs Telem checked"))
    end
    
end


-- Check ESC status
function Check_esc_status_continuous()
    -- check for all ESCs
    for index = 1, MAX_ESC, 1
    do
        -- check for telemetry failure
        -- esc_list[index].ts = esc_telem.get_last_telem_data_ms(index)
        esc_telem.get_last_telem_data_ms(index, esc_list[index].ts)

        if esc_list[index].ts ~= nil then
            if esc_list[index].ts - esc_list[index].prev_ts > ESC_TIMEOUT  and esc_list[index].ts_flag == false then
                gcs:send_text(SEVERITY_CRITICAL, string.format("ESC %d Telemetry Fail", index))
                esc_list[index].ts_flag = true
            else
                esc_list[index].ts_flag = false
            end
            esc_list[index].prev_ts = esc_list[index].ts
        end

        -- check for ESC temperature ranges
        -- esc_list[index].temp = esc_telem.get_temperature(index)
        esc_telem.get_temperature(index, esc_list[index].temp)
        if esc_list[index].temp ~= nil and esc_list[index].temp > 0 then
            if esc_list[index].temp > MAX_ESC_TEMP_THR and esc_list[index].temp_flag == false then
                gcs:send_text(SEVERITY_CRITICAL, string.format("ESC %d Temperature Critical", index))
                esc_list[index].temp_flag = true
            else
                esc_list[index].temp_flag = false
            end
        end

        -- check if rpm is matching with rcout
        Curr_rcout[index] = SRV_Channels:get_output_pwm(RC_out_list[index])
        esc_telem.get_rpm(index, esc_list[index].rpm)
        if esc_list[index].rpm~= nil and esc_list[index].rpm > 0 then
            if Prev_rcout[index] < Curr_rcout[index] then
                if not Prev_esc_rpm[index] < esc_list[index].rpm then
                    gcs:send_text(SEVERITY_CRITICAL, string.format("ESC %d rpm/rcout mismatch", index))
                end
            elseif Prev_rcout[index] > Curr_rcout[index] then
                if not Prev_esc_rpm[index] > esc_list[index].rpm then
                    gcs:send_text(SEVERITY_CRITICAL, string.format("ESC %d rpm/rcout mismatch", index))
                end
            else
                -- do nothing
            end
        end
        Prev_esc_rpm[index] = esc_list[index].rpm
    end
end


-- init code 
function Setup()
    
    Init_check_status()

    return Update, UPDATE_RATE
end


--Main Loop
function Update() -- this is the loop which periodically runs
    -- Check_lua()
    Check_efi_status()
    Check_esc_status()

    return Update, UPDATE_RATE
end


-- wrapper around update(). This calls update() at (1/UPDATE_RATE)Hz,
-- and if update faults then an error is displayed, but the script is not stopped
function Protected_wrapper()

    gcs:send_text(0, "Lua pre-arm check start")

    local success, err = pcall(Update)
    if not success then
       gcs:send_text(0, "Internal Error: " .. err)
       -- when we fault we run the update function again after 1s, slowing it
       -- down a bit so we don't flood the console with errors
       return Protected_wrapper, 1000
       -- return
    end
    return Protected_wrapper, UPDATE_RATE
  end


return Setup, 5000 -- run immediately before starting to reschedule
