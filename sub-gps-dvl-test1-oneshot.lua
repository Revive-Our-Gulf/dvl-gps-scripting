-- One-shot GPS <-> DVL source-set switching test for ArduSub.
--
-- Intent:
--   1. Start on GPS source set
--   2. Switch once to DVL when GPS degrades and DVL is healthy
--   3. Switch straight back to GPS when GPS becomes healthy again
--
-- This is intentionally a literal test of the simple idea. It does not try
-- to suppress position jumps when switching back to GPS.
--
-- Setup:
--   EK3_SRC1_* = GPS based source set
--   EK3_SRC2_* = ExternalNav / DVL source set
--   VISO_TYPE  = 1
--   RC switching is not required; this script drives source selection.
---@diagnostic disable: param-type-mismatch
---@diagnostic disable: need-check-nil

local PARAM_TABLE_KEY = 91
local PARAM_TABLE_PREFIX = "T1_"
local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}
local SOURCE_GPS = 0
local SOURCE_DVL = 1
local RANGEFINDER_ROTATION = 25
local LOOP_MS = 200

local current_source = SOURCE_GPS
local last_report = ""
local gps_bad_votes = 0
local gps_good_votes = 0
local dvl_good_votes = 0

local function bind_param(name)
    local p = Parameter()
    assert(p:init(name), string.format("could not find %s parameter", name))
    return p
end

local function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format("could not add param %s", name))
    return bind_param(PARAM_TABLE_PREFIX .. name)
end

assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 6), "test1: could not add param table")

local T1_GPS_BAD = bind_add_param("GPS_BAD", 1, 0.9)
local T1_GPS_GOOD = bind_add_param("GPS_GOOD", 2, 0.5)
local T1_DVL_QUAL = bind_add_param("DVL_QUAL", 3, 20)
local T1_EXTN_TH = bind_add_param("EXTN_TH", 4, 0.5)
local T1_RNG_MIN = bind_add_param("RNG_MIN", 5, 0.3)
local T1_VOTE_S = bind_add_param("VOTE_S", 6, 2.0)

local function votes_required()
    return math.max(1, math.floor(T1_VOTE_S:get() * (1000.0 / LOOP_MS) + 0.5))
end

local function report(text)
    if text ~= last_report then
        last_report = text
        gcs:send_text(MAV_SEVERITY.INFO, "test1: " .. text)
    end
end

local function gps_health()
    if gps:num_sensors() == 0 then
        return false, nil
    end
    local instance = gps:primary_sensor()
    local status = gps:status(instance)
    if status == nil or status < gps.GPS_OK_FIX_3D then
        return false, nil
    end
    return true, gps:speed_accuracy(instance)
end

local function gps_good()
    local ok, speed_acc = gps_health()
    return ok and speed_acc ~= nil and speed_acc <= T1_GPS_GOOD:get()
end

local function gps_bad()
    local ok, speed_acc = gps_health()
    if not ok then
        return true
    end
    return speed_acc == nil or speed_acc >= T1_GPS_BAD:get()
end

local function rangefinder_good()
    if not rangefinder:has_data_orient(RANGEFINDER_ROTATION) then
        return false
    end
    local distance = rangefinder:distance_orient(RANGEFINDER_ROTATION)
    return distance ~= nil and distance >= T1_RNG_MIN:get()
end

local function dvl_good()
    if visual_odom == nil or not visual_odom:healthy() or visual_odom:quality() < T1_DVL_QUAL:get() then
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
    return innov_len > 0.0 and innov_len <= T1_EXTN_TH:get()
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

    if not arming:is_armed() then
        if current_source ~= SOURCE_GPS then
            switch_source(SOURCE_GPS, "disarmed reset")
        end
        gps_bad_votes = 0
        gps_good_votes = 0
        dvl_good_votes = 0
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

    if gps_good() then
        gps_good_votes = math.min(gps_good_votes + 1, vote_target)
    else
        gps_good_votes = 0
    end

    if current_source == SOURCE_GPS then
        if dvl_good_votes >= vote_target and gps_bad_votes >= vote_target then
            switch_source(SOURCE_DVL, "gps degraded, dvl healthy")
        end
    elseif gps_good_votes >= vote_target then
        switch_source(SOURCE_GPS, "gps recovered")
    end

    return update, LOOP_MS
end

return update()