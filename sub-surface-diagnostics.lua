-- Surface / dive diagnostics for ArduSub.
--
-- Purpose:
--   Help diagnose big track jumps around surfacing/diving by reporting:
--   - active EKF source set
--   - GPS fix and GPS-vs-EKF gap
--   - ExternalNav health / quality
--   - ExternalNav velocity innovation magnitude
--   - sudden EKF position jumps between script updates
--
-- This script is read-only. It does not switch sources or set origin.
---@diagnostic disable: param-type-mismatch
---@diagnostic disable: need-check-nil

local PARAM_TABLE_KEY = 95
local PARAM_TABLE_PREFIX = "SDIAG_"
local TEXT_PREFIX = "surf-diag: "
local FLOAT_PREFIX = "SDI_"
local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}
local EXTNAV_SOURCE = 6
local RANGEFINDER_ROTATION = 25

local last_summary_ms = 0
local last_source = nil
local last_ekf_loc = nil
local last_gap_alert = false
local last_jump_alert = false
local last_innov_alert = false

local function bind_param(name)
    local p = Parameter()
    assert(p:init(name), string.format("could not find %s parameter", name))
    return p
end

local function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format("could not add param %s", name))
    return bind_param(PARAM_TABLE_PREFIX .. name)
end

assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 6), TEXT_PREFIX .. "could not add param table")

local SDIAG_ENABLE = bind_add_param("ENABLE", 1, 1)
local SDIAG_RATEHZ = bind_add_param("RATEHZ", 2, 2)
local SDIAG_TXTPER = bind_add_param("TXTPER", 3, 5)
local SDIAG_GAPWRN = bind_add_param("GAPWRN", 4, 3)
local SDIAG_JUMPTH = bind_add_param("JUMPTH", 5, 2)
local SDIAG_INNTH = bind_add_param("INNTH", 6, 0.5)

local function send_info(text)
    gcs:send_text(MAV_SEVERITY.INFO, TEXT_PREFIX .. text)
end

local function send_warning(text)
    gcs:send_text(MAV_SEVERITY.WARNING, TEXT_PREFIX .. text)
end

local function safe_rate_hz()
    return math.max(1, math.floor(SDIAG_RATEHZ:get() + 0.5))
end

local function update_period_ms()
    return math.floor(1000 / safe_rate_hz())
end

local function text_period_ms()
    return math.max(1000, math.floor(SDIAG_TXTPER:get() * 1000 + 0.5))
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

local function get_gps_snapshot()
    if gps:num_sensors() == 0 then
        return nil
    end
    local instance = gps:primary_sensor()
    return {
        fix = gps:status(instance) or -1,
        hacc = gps:horizontal_accuracy(instance) or -1,
        sacc = gps:speed_accuracy(instance) or -1,
        loc = gps:location(instance)
    }
end

local function get_visual_odom_snapshot()
    if visual_odom == nil then
        return { healthy = false, quality = -1 }
    end
    return {
        healthy = visual_odom:healthy(),
        quality = visual_odom:quality() or -1
    }
end

local function get_extnav_innovation()
    local innov = ahrs:get_vel_innovations_and_variances_for_source(EXTNAV_SOURCE)
    if innov == nil then
        return -1
    end
    return innov:length()
end

local function get_range_snapshot()
    if not rangefinder:has_data_orient(RANGEFINDER_ROTATION) then
        return -1
    end
    return rangefinder:distance_orient(RANGEFINDER_ROTATION) or -1
end

local function get_jump_distance(curr_loc)
    if curr_loc == nil or last_ekf_loc == nil then
        return -1
    end
    return curr_loc:get_distance(last_ekf_loc)
end

local function get_snapshot()
    local ekf_loc = ahrs:get_location()
    local gps_data = get_gps_snapshot()
    local gap = -1
    if ekf_loc ~= nil and gps_data ~= nil and gps_data.fix >= gps.GPS_OK_FIX_3D then
        gap = ekf_loc:get_distance(gps_data.loc)
    end
    return {
        source = ahrs:get_posvelyaw_source_set(),
        origin = ahrs:get_origin(),
        ekf_loc = ekf_loc,
        gps = gps_data,
        vo = get_visual_odom_snapshot(),
        gap = gap,
        jump = get_jump_distance(ekf_loc),
        innov = get_extnav_innovation(),
        rng = get_range_snapshot()
    }
end

local function emit_metrics(snapshot)
    send_metric("SRC", snapshot.source)
    send_metric("ORG", bool_to_num(snapshot.origin ~= nil))
    send_metric("GFIX", snapshot.gps and snapshot.gps.fix or -1)
    send_metric("GAP", snapshot.gap)
    send_metric("VOH", bool_to_num(snapshot.vo.healthy))
    send_metric("VOQ", snapshot.vo.quality)
    send_metric("XINN", snapshot.innov)
    send_metric("JUMP", snapshot.jump)
    send_metric("RNG", snapshot.rng)
end

local function emit_events(snapshot)
    if last_source ~= nil and snapshot.source ~= last_source then
        send_warning(string.format("source changed %d -> %d gap=%.1f innov=%.2f", last_source, snapshot.source, snapshot.gap, snapshot.innov))
    end
    last_source = snapshot.source

    local gap_alert = snapshot.gap >= SDIAG_GAPWRN:get()
    if gap_alert and not last_gap_alert then
        send_warning(string.format("gps/ekf gap high %.1fm src=%d", snapshot.gap, snapshot.source))
    end
    last_gap_alert = gap_alert

    local jump_alert = snapshot.jump >= SDIAG_JUMPTH:get()
    if jump_alert and not last_jump_alert then
        send_warning(string.format("ekf jump %.1fm src=%d gap=%.1f", snapshot.jump, snapshot.source, snapshot.gap))
    end
    last_jump_alert = jump_alert

    local innov_alert = snapshot.innov >= SDIAG_INNTH:get()
    if innov_alert and not last_innov_alert then
        send_warning(string.format("extnav innov high %.2f vo=%d/%d", snapshot.innov, bool_to_num(snapshot.vo.healthy), snapshot.vo.quality))
    end
    last_innov_alert = innov_alert
end

local function emit_summary(snapshot, now_ms)
    if now_ms - last_summary_ms < text_period_ms() then
        return
    end
    last_summary_ms = now_ms

    local gps_fix = snapshot.gps and snapshot.gps.fix or -1
    local gps_hacc = snapshot.gps and snapshot.gps.hacc or -1
    send_info(string.format("src=%d fix=%d gap=%.1f jump=%.1f innov=%.2f vo=%d/%d hacc=%.2f rng=%.1f",
        snapshot.source,
        gps_fix,
        snapshot.gap,
        snapshot.jump,
        snapshot.innov,
        bool_to_num(snapshot.vo.healthy),
        snapshot.vo.quality,
        gps_hacc,
        snapshot.rng))
end

local function update()
    if not ahrs:initialised() then
        return update, 1000
    end
    if SDIAG_ENABLE:get() < 1 then
        return update, 1000
    end

    local now_ms = millis()
    local snapshot = get_snapshot()
    emit_metrics(snapshot)
    emit_events(snapshot)
    emit_summary(snapshot, now_ms)
    last_ekf_loc = snapshot.ekf_loc and snapshot.ekf_loc:copy() or nil

    return update, update_period_ms()
end

send_info("started")
return update()