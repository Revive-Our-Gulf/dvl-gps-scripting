-- Jump-aware GPS <-> DVL handover test for ArduSub.
--
-- Intent:
--   1. Bootstrap origin from GPS if needed.
--   2. Start on GPS source set.
--   3. Switch to DVL only after DVL quality and innovations are stable.
--   4. Switch back to GPS only when raw GPS and current EKF position already
--      agree within a configured threshold.
--
-- This is the recommended script of the three for minimizing jumps during
-- surface/submerged handoff testing.
---@diagnostic disable: param-type-mismatch
---@diagnostic disable: need-check-nil

local PARAM_TABLE_KEY = 93
local PARAM_TABLE_PREFIX = "TG_"
local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}
local SOURCE_GPS = 0
local SOURCE_DVL = 1
local RANGEFINDER_ROTATION = 25
local LOOP_MS = 200

local current_source = SOURCE_GPS
local last_report = ""
local gps_bad_votes = 0
local dvl_good_votes = 0
local gps_return_votes = 0
local origin_reported = false

local function bind_param(name)
    local p = Parameter()
    assert(p:init(name), string.format("could not find %s parameter", name))
    return p
end

local function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format("could not add param %s", name))
    return bind_param(PARAM_TABLE_PREFIX .. name)
end

assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 8), "gated: could not add param table")

local TG_GPS_BAD = bind_add_param("GPS_BAD", 1, 0.8)
local TG_GPS_GOOD = bind_add_param("GPS_GOOD", 2, 0.5)
local TG_DVL_QUAL = bind_add_param("DVL_QUAL", 3, 20)
local TG_EXTN_TH = bind_add_param("EXTN_TH", 4, 0.5)
local TG_ALIGN_M = bind_add_param("ALIGN_M", 5, 2.0)
local TG_RNG_MIN = bind_add_param("RNG_MIN", 6, 0.3)
local TG_RNG_MAX = bind_add_param("RNG_MAX", 7, 50.0)
local TG_VOTE_S = bind_add_param("VOTE_S", 8, 2.0)

local function votes_required()
    return math.max(1, math.floor(TG_VOTE_S:get() * (1000.0 / LOOP_MS) + 0.5))
end

local function report(text)
    if text ~= last_report then
        last_report = text
        gcs:send_text(MAV_SEVERITY.INFO, "gated: " .. text)
    end
end

local function gps_snapshot()
    if gps:num_sensors() == 0 then
        return false, nil, nil
    end
    local instance = gps:primary_sensor()
    local status = gps:status(instance)
    local speed_acc = gps:speed_accuracy(instance)
    if status == nil or status < gps.GPS_OK_FIX_3D then
        return false, speed_acc, nil
    end
    return true, speed_acc, gps:location(instance)
end

local function gps_good()
    local ok, speed_acc, gps_loc = gps_snapshot()
    return ok and speed_acc ~= nil and speed_acc <= TG_GPS_GOOD:get(), gps_loc
end

local function gps_bad()
    local ok, speed_acc = gps_snapshot()
    if not ok then
        return true
    end
    return speed_acc == nil or speed_acc >= TG_GPS_BAD:get()
end

local function rangefinder_good()
    if not rangefinder:has_data_orient(RANGEFINDER_ROTATION) then
        return false
    end
    local distance = rangefinder:distance_orient(RANGEFINDER_ROTATION)
    return distance ~= nil and distance >= TG_RNG_MIN:get() and distance <= TG_RNG_MAX:get()
end

local function dvl_good()
    if visual_odom == nil or not visual_odom:healthy() or visual_odom:quality() < TG_DVL_QUAL:get() then
        return false
    end
    if not rangefinder_good() then
        return false
    end
    local innov = ahrs:get_vel_innovations_and_variances_for_source(6)
    if innov == nil then
        return false
    end
    local innov_len = innov:length()
    return innov_len > 0.0 and innov_len <= TG_EXTN_TH:get()
end

local function current_gps_gap_m()
    local ekf_loc = ahrs:get_location()
    local ok, gps_loc = gps_good()
    if not ok or ekf_loc == nil or gps_loc == nil then
        return nil
    end
    return ekf_loc:get_distance(gps_loc)
end

local function maybe_set_origin_once()
    if ahrs:get_origin() then
        return
    end
    local ok, gps_loc = gps_good()
    if not ok or gps_loc == nil then
        return
    end
    if ahrs:set_origin(gps_loc:copy()) then
        if not origin_reported then
            report("origin bootstrapped from GPS")
            origin_reported = true
        end
    end
end

local function switch_source(new_source, why)
    if current_source == new_source then
        return
    end
    current_source = new_source
    ahrs:set_posvelyaw_source_set(new_source)
    report(string.format("source=%u (%s)", new_source + 1, why))
end

function update()
    if not ahrs:initialised() then
        return update, 1000
    end

    maybe_set_origin_once()

    if not arming:is_armed() then
        if current_source ~= SOURCE_GPS then
            switch_source(SOURCE_GPS, "disarmed reset")
        end
        gps_bad_votes = 0
        dvl_good_votes = 0
        gps_return_votes = 0
        return update, LOOP_MS
    end

    local vote_target = votes_required()

    if dvl_good() then
        dvl_good_votes = math.min(dvl_good_votes + 1, vote_target)
    else
        dvl_good_votes = 0
    end

    if gps_bad() then
        gps_bad_votes = math.min(gps_bad_votes + 1, vote_target)
    else
        gps_bad_votes = 0
    end

    if current_source == SOURCE_GPS then
        if dvl_good_votes >= vote_target and gps_bad_votes >= vote_target then
            switch_source(SOURCE_DVL, "gps degrading, dvl stable")
        end
        return update, LOOP_MS
    end

    local gps_ok = gps_good()
    local gap_m = current_gps_gap_m()
    if gps_ok and gap_m ~= nil and gap_m <= TG_ALIGN_M:get() then
        gps_return_votes = math.min(gps_return_votes + 1, vote_target)
    else
        gps_return_votes = 0
    end

    if gps_return_votes >= vote_target then
        switch_source(SOURCE_GPS, string.format("gps aligned %.1fm", gap_m))
    elseif gps_ok and gap_m ~= nil then
        report(string.format("holding DVL, gps gap %.1fm", gap_m))
    end

    return update, LOOP_MS
end

return update()