local hdop_threshold = 80
local hdop_below_threshold_time = 0 
local threshold_time_max = 1000
local loop_delay = 100

local gps_sensor = gps:primary_sensor()
local location_queue = {}

function get_gps_data(instance)
    local data = {
        hdop = gps:get_hdop(instance),
        satellites = gps:num_sats(instance),
        location = gps:location(instance),
        status = gps:status(instance)
    }
    if data.status < gps.GPS_OK_FIX_3D then
        return nil
    end
    return data
end

function reset_state()
    hdop_below_threshold_time = 0
    location_queue = {}
end

function update()
    location_queue = location_queue or {}

    if not ahrs:initialised() then
        return update, 5000
    end

    gps_data = get_gps_data(gps_sensor)

    if gps_data == nil then
        reset_state()
        return update, loop_delay
    end

    if gps_data.hdop < hdop_threshold then
        hdop_below_threshold_time = hdop_below_threshold_time + 100
        table.insert(location_queue, {
            lat = gps_data.location.lat,
            lng = gps_data.location.lng,
            hdop = gps_data.hdop
        })
    else
        reset_state()
        return update, loop_delay
    end

    if hdop_below_threshold_time >= threshold_time_max then
        if gps_data then
            local sum_lat = 0
            local sum_lng = 0
            for _, loc in ipairs(location_queue) do
                sum_lat = sum_lat + loc.lat
                sum_lng = sum_lng + loc.lng
            end

            local average_lat = sum_lat / #location_queue
            local average_lng = sum_lng / #location_queue

            local ahrs_location = ahrs:get_origin()
        
            ahrs_location.lat = average_lat
            ahrs_location.lng = average_lng

            ahrs:set_origin(ahrs_location)
            gcs:send_text(6, string.format("AHRS origin has been set by Script to lat: %.6f, lng: %.6f", average_lat, average_lng))
        end

        reset_state()
    end

    return update, loop_delay
end

return update()