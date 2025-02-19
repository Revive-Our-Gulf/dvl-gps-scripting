-- HDOP, lower is better. Lower threshold value means GPS needs to be more accurate
-- Avoid setting too low. HDOP might never reach value. Noise might be too high
local hdop_threshold = 80
local hdop_below_threshold_time = 0 
local threshold_time_max = 1000
local loop_delay = 100 -- Ensure threshold_time_max / loop_delay is int

local gps_sensor = gps:primary_sensor()
local sum_location = {
    lat = 0,
    lng = 0
} 


function get_gps_data(instance)
    local data = {
        hdop = gps:get_hdop(instance),
        satellites = gps:num_sats(instance),
        location = gps:location(instance),
        status = gps:status(instance)
    }
    if data.status < gps.GPS_OK_FIX_3D then
        return nil -- don't bother with invalid data
    end
    return data
end

function update()

    -- wait for AHRS to be initialised
    -- Might not be needed, included for startup, i.e incase rebooted underwater
    if not ahrs:initialised() then
        return update, 5000
    end

    gps_data = get_gps_data(gps_sensor)

    if gps_data == nil then
        hdop_below_threshold_time = 0 
        return update, loop_delay
    end

    if gps_data.hdop < hdop_threshold then
        hdop_below_threshold_time = hdop_below_threshold_time + 100
        sum_location.lat = sum_location.lat + gps_data.location.lat
        sum_location.lng = sum_location.lng + gps_data.location.lng
    else
        hdop_below_threshold_time = 0
        sum_location.lat = 0
        sum_location.lng = 0
        return update, loop_delay
    end

    if hdop_below_threshold_time >= threshold_time_max then
        if gps_data then
            local average_lat = sum_location.lat / (threshold_time_max / loop_delay)
            local average_lng = sum_location.lng / (threshold_time_max / loop_delay)

            local ahrs_location = ahrs:get_origin()
            ahrs_location.lat = average_lat
            ahrs_location.lng = average_lng

            ahrs:set_origin(ahrs_location)
            gcs:send_text(6, string.format("AHRS origin has been set by Script to lat: %.6f, lng: %.6f", average_lat, average_lng))
        end

        -- Reset timer
        -- If origin set, will wait 'threshold_time_max' till setting again
        hdop_below_threshold_time = 0

        sum_location.lat = 0
        sum_location.lng = 0
    end
    

    return update, loop_delay
end

return update()
