-- Monitor-only navigation diagnostics for ArduSub.
--
-- This script is intended as a first-step diagnostic tool. It does not change
-- source sets or origin. It only reports current navigation state through
-- STATUSTEXT and NAMED_VALUE_FLOAT messages.
---@diagnostic disable: param-type-mismatch
---@diagnostic disable: need-check-nil

local PARAM_TABLE_KEY = 94
local PARAM_TABLE_PREFIX = "NDIAG_"
local SCRIPT_PREFIX = "nav-diag: "
local FLOAT_PREFIX = "NDI_"
local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

local last_summary_ms = 0
local last_gps_warning = false
local last_origin_warning = false
local last_ekf_warning = false

local function bind_param(name)
    local p = Parameter()
    assert(p:init(name), string.format("could not find %s parameter", name))
    return p
end

local function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format("could not add param %s", name))
    return bind_param(PARAM_TABLE_PREFIX .. name)
end

assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 7), SCRIPT_PREFIX .. "could not add param table")

local NDIAG_ENABLE = bind_add_param("ENABLE", 1, 1)
local NDIAG_RATEHZ = bind_add_param("RATEHZ", 2, 2)
local NDIAG_VERBOSE = bind_add_param("VERBOSE", 3, 1)
local NDIAG_TXTPER = bind_add_param("TXTPER", 4, 5)
local NDIAG_MANSRC = bind_add_param("MANSRC", 5, -1)
local NDIAG_MANGAP = bind_add_param("MANGAP", 6, -1)
local NDIAG_USER1 = bind_add_param("USER1", 7, 0)

local function send_info(text)
    gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_PREFIX .. text)
end

local function send_warning(text)
    gcs:send_text(MAV_SEVERITY.WARNING, SCRIPT_PREFIX .. text)
end

local function safe_rate_hz()
    return math.max(1, math.floor(NDIAG_RATEHZ:get() + 0.5))
end

local function update_period_ms()
    return math.floor(1000 / safe_rate_hz())
end

local function text_period_ms()
    return math.max(1000, math.floor(NDIAG_TXTPER:get() * 1000 + 0.5))
end

local function bool_to_num(value)
    if value then
        return 1
    end
    return 0
end

local function send_metric(name, value)
    gcs:send_named_float(FLOAT_PREFIX .. name, value)
end

local function display_source(source)
    if NDIAG_MANSRC:get() >= 0 then
        return NDIAG_MANSRC:get()
    end
    return source
end

local function display_gap(gap)
    if NDIAG_MANGAP:get() >= 0 then
        return NDIAG_MANGAP:get()
    end
    return gap
end

local function get_gps_snapshot()
    if gps:num_sensors() == 0 then
        return nil
    end

    local instance = gps:primary_sensor()
    return {
        instance = instance,
        fix = gps:status(instance) or -1,
        hacc = gps:horizontal_accuracy(instance) or -1,
        sacc = gps:speed_accuracy(instance) or -1,
        loc = gps:location(instance)
    }
end

local function get_visual_odom_snapshot()
    if visual_odom == nil then
        return {
            healthy = false,
            quality = -1
        }
    end

    return {
        healthy = visual_odom:healthy(),
        quality = visual_odom:quality() or -1
    }
end

local function get_nav_snapshot()
    local ekf_loc = ahrs:get_location()
    local origin = ahrs:get_origin()
    local gps_data = get_gps_snapshot()
    local vo_data = get_visual_odom_snapshot()
    local gps_gap = -1

    if gps_data ~= nil and ekf_loc ~= nil and gps_data.fix >= gps.GPS_OK_FIX_3D then
        gps_gap = ekf_loc:get_distance(gps_data.loc)
    end

    return {
        source = ahrs:get_posvelyaw_source_set(),
        ekf_loc = ekf_loc,
        origin = origin,
        gps = gps_data,
        vo = vo_data,
        gps_gap = gps_gap
    }
end

local function emit_metrics(snapshot)
    send_metric("SRC", display_source(snapshot.source))
    send_metric("ORG", bool_to_num(snapshot.origin ~= nil))
    send_metric("VOH", bool_to_num(snapshot.vo.healthy))
    send_metric("VOQ", snapshot.vo.quality)
    send_metric("U1", NDIAG_USER1:get())

    if snapshot.gps == nil then
        send_metric("GFIX", -1)
        send_metric("HACC", -1)
        send_metric("SACC", -1)
        send_metric("GAP", display_gap(-1))
        return
    end

    send_metric("GFIX", snapshot.gps.fix)
    send_metric("HACC", snapshot.gps.hacc)
    send_metric("SACC", snapshot.gps.sacc)
    send_metric("GAP", display_gap(snapshot.gps_gap))
end

local function emit_warnings(snapshot)
    local gps_missing = snapshot.gps == nil
    if gps_missing and not last_gps_warning then
        send_warning("no GPS sensors detected")
    end
    last_gps_warning = gps_missing

    local origin_missing = snapshot.origin == nil
    if origin_missing and not last_origin_warning then
        send_warning("EKF origin unset")
    end
    last_origin_warning = origin_missing

    local ekf_missing = snapshot.ekf_loc == nil
    if ekf_missing and not last_ekf_warning then
        send_warning("EKF location unavailable")
    end
    last_ekf_warning = ekf_missing
end

local function emit_summary(snapshot, now_ms)
    if NDIAG_VERBOSE:get() < 1 then
        return
    end

    if now_ms - last_summary_ms < text_period_ms() then
        return
    end
    last_summary_ms = now_ms

    local gps_fix = -1
    local gps_hacc = -1
    local gap_text = "n/a"

    if snapshot.gps ~= nil then
        gps_fix = snapshot.gps.fix
        gps_hacc = snapshot.gps.hacc
    end

    local shown_source = display_source(snapshot.source)
    local shown_gap = display_gap(snapshot.gps_gap)

    if shown_gap >= 0 then
        gap_text = string.format("%.1f", shown_gap)
    end

    send_info(string.format("src=%d gps_fix=%d hacc=%.2f vo=%d/%d origin=%d gap=%s",
        shown_source,
        gps_fix,
        gps_hacc,
        bool_to_num(snapshot.vo.healthy),
        snapshot.vo.quality,
        bool_to_num(snapshot.origin ~= nil),
        gap_text))
end

local function update()
    if not ahrs:initialised() then
        return update, 1000
    end

    if NDIAG_ENABLE:get() < 1 then
        return update, 1000
    end

    local now_ms = millis()
    local snapshot = get_nav_snapshot()

    emit_metrics(snapshot)
    emit_warnings(snapshot)
    emit_summary(snapshot, now_ms)

    return update, update_period_ms()
end

send_info("started")
return update()