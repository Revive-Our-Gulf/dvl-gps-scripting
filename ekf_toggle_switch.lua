    -- Wanted source:
    -- 0 = EK3_SRC1 (External NAV)
    -- 1 = EK3_SRC2 (GPS)

    -- Parameters:
    local hdop_threshold = 80
    local haccuracy_threshold = 3

    
    local PARAM_TABLE_KEY = 90
    local PARAM_TABLE_PREFIX = 'DVL_'

    -- bind a parameter to a variable
    function bind_param(name)
    local p = Parameter()
    assert(p:init(name), string.format('could not find %s parameter', name))
    return p
    end

    -- add a parameter and bind it to a variable
    function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
    return bind_param(PARAM_TABLE_PREFIX .. name)
    end

    assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 32), 'could not add param table')

    local hdop_threshold = bind_add_param('HDOP_THR', 1, 80)

    -- Setup ekf sources
    local ek3_src1_posxy = bind_param('EK3_SRC1_POSXY')
    local ek3_src1_velxy = bind_param('EK3_SRC1_VELXY')
    local ek3_src1_posz = bind_param('EK3_SRC1_POSZ')
    local ek3_src1_velz = bind_param('EK3_SRC1_VELZ')

    -- TODO: do not change this, just alert the user
    ek3_src1_posxy:set(6)
    ek3_src1_velxy:set(6)
    ek3_src1_posz:set(1)
    ek3_src1_velz:set(0)

    local ek3_src2_posxy = bind_param('EK3_SRC2_POSXY')
    local ek3_src2_velxy = bind_param('EK3_SRC2_VELXY')
    local ek3_src2_posz = bind_param('EK3_SRC2_POSZ')
    local ek3_src2_velz = bind_param('EK3_SRC2_VELZ')

    -- TODO: do not change this, just alert the user
    ek3_src2_posxy:set(3)
    ek3_src2_velxy:set(6)
    ek3_src2_posz:set(1)
    ek3_src2_velz:set(0)

    local options = bind_param('EK3_SRC_OPTIONS')
    options:set(0)

    function get_gps_data(gps_sensor)
        local gps_hdop = gps:get_hdop(gps_sensor)
        local gps_haccuracy = gps:horizontal_accuracy(gps_sensor)
        local gps_status = gps:status(gps_sensor)
        local gps_sats = gps:num_sats(gps_sensor)

        if gps_hdop == nil then
            gps_hdop = 9999
        end

        if gps_haccuracy == nil then
            gps_haccuracy = 9999
        end

        if gps_status == nil then
            gps_status = -1
        end

        if gps_sats == nil then
            gps_sats = -1
        end

        return {
            hdop = gps_hdop,
            haccuracy = gps_haccuracy,
            status = gps_status,
            sats = gps_sats
        }
    end

    function update()

        gps_data = get_gps_data(gps:primary_sensor())
        
        local wanted_source = 0
        if gps_data.hdop <= 80 and gps_data.haccuracy <= 3 and gps_data.status > 1 then
            wanted_source = 1
        end

        if ahrs:get_posvelyaw_source_set() ~= wanted_source then
            if wanted_source == 0 then
                gcs:send_text(6, string.format("Switched to DVL, HAccuracy: %.2f, HDOP: %.2f", gps_data.haccuracy, gps_data.hdop))
            else
                gcs:send_text(6, string.format("Switched to GPS, HAccuracy: %.2f, HDOP: %.2f", gps_data.haccuracy, gps_data.hdop))
            end
            ahrs:set_posvelyaw_source_set(wanted_source)
        end

        gcs:send_named_float('EkfSource', ahrs:get_posvelyaw_source_set())
        
        if not gps_data.hdop then
            gcs:send_text(6, "Error: Unable to retrieve GPS HDOP")
        end
        if not gps_data.haccuracy then
            gcs:send_text(6, "Error: Unable to retrieve GPS Horizontal Accuracy")
        end
        if not gps_data.status then
            gcs:send_text(6, "Error: Unable to retrieve GPS Status")
        end
        if not gps_data.sats then
            gcs:send_text(6, "Error: Unable to retrieve GPS Satellites")
        end

        return update, 200
    end

    return update()
