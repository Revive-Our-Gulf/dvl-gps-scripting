-- Test 2 bootstrap script for ArduSub.
--
-- Intent:
--   Run ExternalNav / DVL as the only EKF horizontal source while still
--   reading GPS in Lua. If EKF origin is unset, set it once from GPS when GPS
--   and DVL look healthy.
--
-- Important limitation:
--   This script does not try to repeatedly change origin. ArduPilot accepts
--   origin setting only while origin is still unset.
---@diagnostic disable: param-type-mismatch
---@diagnostic disable: need-check-nil

local PARAM_TABLE_KEY = 92
local PARAM_TABLE_PREFIX = "T2_"
local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}
local RANGEFINDER_ROTATION = 25
local LOOP_MS = 500

local last_report = ""
local origin_set_reported = false
local bootstrap_votes = 0

local function bind_param(name)
    local p = Parameter()
    assert(p:init(name), string.format("could not find %s parameter", name))
    return p
end

local function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format("could not add param %s", name))
    return bind_param(PARAM_TABLE_PREFIX .. name)
end

assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 6), "test2: could not add param table")

local T2_GPS_GOOD = bind_add_param("GPS_GOOD", 1, 0.5)
local T2_DVL_QUAL = bind_add_param("DVL_QUAL", 2, 20)
local T2_EXTN_TH = bind_add_param("EXTN_TH", 3, 0.5)
local T2_RNG_MIN = bind_add_param("RNG_MIN", 4, 0.3)
local T2_RNG_MAX = bind_add_param("RNG_MAX", 5, 50.0)
local T2_VOTE_S = bind_add_param("VOTE_S", 6, 2.0)

local function votes_required()
    return math.max(1, math.floor(T2_VOTE_S:get() * (1000.0 / LOOP_MS) + 0.5))
end

local function report(text, severity)
    if text ~= last_report then
        last_report = text
        gcs:send_text(severity or MAV_SEVERITY.INFO, "test2: " .. text)
    end
end

local function gps_good()
    if gps:num_sensors() == 0 then
        return false, nil
    end
    local instance = gps:primary_sensor()
    local status = gps:status(instance)
    local speed_acc = gps:speed_accuracy(instance)
    if status == nil or status < gps.GPS_OK_FIX_3D then
        return false, nil
    end
    if speed_acc == nil or speed_acc > T2_GPS_GOOD:get() then
        return false, nil
    end
    return true, gps:location(instance)
end

local function dvl_good()
    if visual_odom == nil or not visual_odom:healthy() or visual_odom:quality() < T2_DVL_QUAL:get() then
        return false
    end
    if not rangefinder:has_data_orient(RANGEFINDER_ROTATION) then
        return false
    end
    local distance = rangefinder:distance_orient(RANGEFINDER_ROTATION)
    if distance == nil or distance < T2_RNG_MIN:get() or distance > T2_RNG_MAX:get() then
        return false
    end
    local innov = ahrs:get_vel_innovations_and_variances_for_source(6)
    if innov == nil then
        return false
    end
    local innov_len = innov:length()
    return innov_len > 0.0 and innov_len <= T2_EXTN_TH:get()
end

function update()
    if not ahrs:initialised() then
        return update, 1000
    end

    local existing_origin = ahrs:get_origin()
    if existing_origin then
        if not origin_set_reported then
            origin_set_reported = true
            report("origin already set; bootstrap path inactive")
        end
        return update, LOOP_MS
    end

    local gps_ok, gps_loc = gps_good()
    local dvl_ok = dvl_good()

    if gps_ok and dvl_ok then
        bootstrap_votes = math.min(bootstrap_votes + 1, votes_required())
    else
        bootstrap_votes = 0
    end

    if bootstrap_votes < votes_required() then
        return update, LOOP_MS
    end

    local origin_loc = gps_loc:copy()
    if ahrs:set_origin(origin_loc) then
        report(string.format("origin set from GPS %.7f %.7f", origin_loc:lat() * 1.0e-7, origin_loc:lng() * 1.0e-7))
        origin_set_reported = true
    else
        report("origin set failed", MAV_SEVERITY.WARNING)
    end

    return update, LOOP_MS
end

return update()