-- HDOP, lower is better. Lower threshold value means GPS needs to be more accurate
-- Avoid setting too low. HDOP might never reach value. Noise might be too high
local hdop_threshold = 80
local hdop_below_threshold_time = 0 
local threshold_time_max = 1000

local gps_sensor = gps:primary_sensor()


function get_gps_data(instance)
    local data = {
        hdop = gps:get_hdop(instance),
        satellites = gps:num_sats(instance),
        location = gps:location(instance),
        status = gps:status(instance),
    }
    if data.status < gps.GPS_OK_FIX_3D then
        return nil -- don't bother with invalid data
    end
    return data
end

function update()

    -- wait for AHRS to be initialised
    -- Might not be needed, including for startup, i.e incase rebooted underwater
    if not ahrs:initialised() then
        return update, 5000
    end

    gps_data = get_gps_data(gps_sensor)

    if gps_data == nil then
        hdop_below_threshold_time = 0 
        return update, 100
    end

    if gps_data.hdop < hdop_threshold then
        hdop_below_threshold_time = hdop_below_threshold_time + 100
    else
        hdop_below_threshold_time = 0
        return update, 100
    end

    if hdop_below_threshold_time >= threshold_time_max then
        if gps_data then
                ahrs:set_origin(gps_data.location)
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
