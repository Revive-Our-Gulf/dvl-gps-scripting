-- HDOP, lower is better. Lower threshold value means GPS needs to be more accurate
-- Avoid setting too low. HDOP might never reach value. Noise might be too high
local hdop_threshold = 80
local hdop_below_threshold_time = 0 
local threshold_time_max = 1000

function update()

    -- wait for AHRS to be initialised
    -- Might not be needed, including for startup, i.e incase rebooted underwater
    if not ahrs:initialised() then
        return update, 5000
    end

    local location = ahrs:get_location()
    local gps_hdop = gps:get_hdop(gps:primary_sensor())

    if gps_hdop == nil then
        hdop_below_threshold_time = 0 
        return update, 100
    end

    if gps_hdop < hdop_threshold then
        hdop_below_threshold_time = hdop_below_threshold_time + 100
    else
        hdop_below_threshold_time = 0
        return update, 100
    end

    if hdop_below_threshold_time >= threshold_time_max then
        if not ahrs:get_origin() and location then
                ahrs:set_origin(location)
        end

        -- Reset timer
        -- If origin set, will wait 'threshold_time_max' till setting again
        hdop_below_threshold_time = 0
    end

    gcs:send_text(6, "Current HDOP: " .. tostring(gps_hdop))
    gcs:send_text(6, "HDOP below threshold time: " .. tostring(hdop_below_threshold_time))

    return update, 100
end

return update()
