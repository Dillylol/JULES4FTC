package org.firstinspires.ftc.teamcode.jules.shot;

import androidx.annotation.Nullable;

import com.google.gson.JsonElement;
import com.google.gson.JsonObject;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.jules.bridge.JulesStreamBus;
import org.firstinspires.ftc.teamcode.jules.bridge.util.GsonCompat;

import java.util.ArrayDeque;
import java.util.HashSet;
import java.util.Locale;
import java.util.Set;

/**
 * Bridge helper that hides the transport-layer specifics for the shot planner protocol.
 */
public final class ShotPlannerBridge {

    public static final class PoseSnapshot {
        public final double xIn;
        public final double yIn;
        public final double headingDeg;

        public PoseSnapshot(double xIn, double yIn, double headingDeg) {
            this.xIn = xIn;
            this.yIn = yIn;
            this.headingDeg = headingDeg;
        }
    }

    public static final class ShotContext {
        public final PoseSnapshot pose;
        public final double rangeIn;
        public final double vBattLoad;
        public final double rpmMeasured;
        public final double rpmBase;
        public final double rpmTargetCmd;
        public final double headingToTagDeg;

        public ShotContext(PoseSnapshot pose,
                           double rangeIn,
                           double vBattLoad,
                           double rpmMeasured,
                           double rpmBase,
                           double rpmTargetCmd,
                           double headingToTagDeg) {
            this.pose = pose;
            this.rangeIn = rangeIn;
            this.vBattLoad = vBattLoad;
            this.rpmMeasured = rpmMeasured;
            this.rpmBase = rpmBase;
            this.rpmTargetCmd = rpmTargetCmd;
            this.headingToTagDeg = headingToTagDeg;
        }
    }

    public static final class ShotToken {
        public final String trialId;
        public final long timestampMs;
        public final double rangeIn;
        public final double vBattLoad;
        public final double rpmTargetCmd;
        public final double rpmAtFire;
        public final long timeToReadyMs;
        public final PoseSnapshot pose;
        public final double headingToTagDeg;

        public ShotToken(String trialId,
                         long timestampMs,
                         double rangeIn,
                         double vBattLoad,
                         double rpmTargetCmd,
                         double rpmAtFire,
                         long timeToReadyMs,
                         PoseSnapshot pose,
                         double headingToTagDeg) {
            this.trialId = trialId;
            this.timestampMs = timestampMs;
            this.rangeIn = rangeIn;
            this.vBattLoad = vBattLoad;
            this.rpmTargetCmd = rpmTargetCmd;
            this.rpmAtFire = rpmAtFire;
            this.timeToReadyMs = timeToReadyMs;
            this.pose = pose;
            this.headingToTagDeg = headingToTagDeg;
        }
    }

    private static final String TAG = "ShotPlannerBridge";
    private static final int MAX_SEEN_CMDS = 64;

    private final JulesStreamBus streamBus;
    private final ArrayDeque<String> recentCmdIds = new ArrayDeque<>();
    private final Set<String> recentCmdSet = new HashSet<>();

    private String sessionId = "";
    private long obsSeq = 0L;
    private long tokenSeq = 0L;

    public ShotPlannerBridge(@Nullable JulesStreamBus streamBus) {
        this.streamBus = streamBus;
    }

    public void setSessionId(String sessionId) {
        this.sessionId = (sessionId != null) ? sessionId : "";
    }

    public void sendHello(String sessionId, long nowMs) {
        setSessionId(sessionId);
        if (streamBus == null) {
            return;
        }
        String json = String.format(Locale.US,
                "{\"type\":\"hello\",\"session_id\":\"%s\",\"ts\":%d}",
                this.sessionId,
                nowMs);
        streamBus.publishJsonLine(json);
    }

    public void sendObservation(ShotContext context, long nowMs) {
        if (streamBus == null || context == null || context.pose == null) {
            return;
        }
        String json = String.format(Locale.US,
                "{\"type\":\"obs\",\"session_id\":\"%s\",\"seq\":%d,\"ts\":%d,"
                        + "\"pose\":{\"x\":%.2f,\"y\":%.2f,\"h\":%.2f},"
                        + "\"range_in\":%.2f,\"v_batt_load\":%.2f,\"rpm_meas\":%.1f,"
                        + "\"rpm_base\":%.1f,\"rpm_tgt_cmd\":%.1f,\"heading_to_tag_deg\":%.2f}",
                sessionId,
                ++obsSeq,
                nowMs,
                context.pose.xIn,
                context.pose.yIn,
                context.pose.headingDeg,
                context.rangeIn,
                context.vBattLoad,
                context.rpmMeasured,
                context.rpmBase,
                context.rpmTargetCmd,
                context.headingToTagDeg);
        streamBus.publishJsonLine(json);
    }

    public void sendRequestShotPlan(ShotContext context, long nowMs) {
        if (streamBus == null || context == null || context.pose == null) {
            return;
        }
        String json = String.format(Locale.US,
                "{\"type\":\"request_shot_plan\",\"session_id\":\"%s\",\"seq\":%d,\"ts\":%d,"
                        + "\"pose\":{\"x\":%.2f,\"y\":%.2f,\"h\":%.2f},"
                        + "\"range_in\":%.2f,\"v_batt_load\":%.2f,\"rpm_meas\":%.1f,"
                        + "\"rpm_base\":%.1f,\"rpm_tgt_cmd\":%.1f,\"heading_to_tag_deg\":%.2f}",
                sessionId,
                ++obsSeq,
                nowMs,
                context.pose.xIn,
                context.pose.yIn,
                context.pose.headingDeg,
                context.rangeIn,
                context.vBattLoad,
                context.rpmMeasured,
                context.rpmBase,
                context.rpmTargetCmd,
                context.headingToTagDeg);
        streamBus.publishJsonLine(json);
    }

    public void sendShotToken(ShotToken token) {
        if (streamBus == null || token == null || token.pose == null) {
            return;
        }
        String json = String.format(Locale.US,
                "{\"type\":\"token:shot_fired\",\"session_id\":\"%s\",\"seq\":%d,\"trial_id\":\"%s\","
                        + "\"ts\":%d,\"range_in\":%.2f,\"v_batt_load\":%.2f,"
                        + "\"rpm_tgt_cmd\":%.1f,\"rpm_at_fire\":%.1f,\"t_to_ready_ms\":%d,"
                        + "\"pose\":{\"x\":%.2f,\"y\":%.2f,\"h\":%.2f},"
                        + "\"heading_to_tag_deg\":%.2f}",
                sessionId,
                ++tokenSeq,
                token.trialId,
                token.timestampMs,
                token.rangeIn,
                token.vBattLoad,
                token.rpmTargetCmd,
                token.rpmAtFire,
                token.timeToReadyMs,
                token.pose.xIn,
                token.pose.yIn,
                token.pose.headingDeg,
                token.headingToTagDeg);
        streamBus.publishJsonLine(json);
    }

    /** Attempt to parse a planner command; returns {@code null} when ignored. */
    @Nullable
    public ShotPlan tryParseShotPlan(JsonObject payload, long nowMs) {
        if (payload == null) {
            return null;
        }
        String type = optString(payload, "type");
        if (type != null && !"cmd".equalsIgnoreCase(type) && !"shot_plan".equalsIgnoreCase(type)) {
            return null;
        }

        String cmdId = optString(payload, "cmd_id");
        if (cmdId == null || cmdId.isEmpty()) {
            RobotLog.w(TAG, "Dropping planner command without cmd_id");
            return null;
        }
        if (recentCmdSet.contains(cmdId)) {
            RobotLog.v(TAG, "Ignoring duplicate planner command %s", cmdId);
            return null;
        }

        long validMs = optLong(payload, "valid_ms", -1L);
        long sentMs = optLong(payload, "sent_ms", optLong(payload, "timestamp_ms", optLong(payload, "ts", -1L)));
        if (validMs <= 0L || sentMs < 0L) {
            RobotLog.w(TAG, "Dropping command %s missing TTL", cmdId);
            return null;
        }
        long validUntil = sentMs + validMs;
        if (nowMs > validUntil) {
            RobotLog.v(TAG, "Command %s expired (now=%d > %d)", cmdId, nowMs, validUntil);
            return null;
        }

        Double rpmBias = optNullableDouble(payload, "rpm_bias");
        Double rpmAbs = optNullableDouble(payload, "rpm_target_abs");
        if ((rpmBias == null && rpmAbs == null) || (rpmBias != null && rpmAbs != null)) {
            RobotLog.w(TAG, "Command %s missing or conflicting RPM directive", cmdId);
            return null;
        }

        double rangeDeltaIn = optDouble(payload, "range_delta_in", 0.0);
        boolean loiter = optBoolean(payload, "loiter", false);
        double headingOffset = optDouble(payload, "heading_offset_deg", 0.0);
        String trialId = optString(payload, "trial_id");
        if (trialId == null || trialId.isEmpty()) {
            trialId = cmdId;
        }

        rememberCommand(cmdId);
        return new ShotPlan(cmdId,
                trialId,
                rpmBias,
                rpmAbs,
                rangeDeltaIn,
                loiter,
                headingOffset,
                sentMs,
                validUntil,
                nowMs);
    }

    public boolean isModelUpdate(JsonObject payload) {
        if (payload == null) {
            return false;
        }
        String type = optString(payload, "type");
        return type != null && "rpm_model_update".equalsIgnoreCase(type);
    }

    /** Parse a raw command string into a JSON object, handling mailbox wrappers. */
    @Nullable
    public static JsonObject unwrap(String raw) {
        if (raw == null || raw.isEmpty()) {
            return null;
        }
        JsonElement element = GsonCompat.parse(raw);
        if (element == null || !element.isJsonObject()) {
            return null;
        }
        JsonObject obj = element.getAsJsonObject();
        String type = optString(obj, "type");
        if ("cmd".equalsIgnoreCase(type) && obj.has("text")) {
            JsonElement textElement = obj.get("text");
            if (textElement.isJsonObject()) {
                return textElement.getAsJsonObject();
            }
            if (textElement.isJsonPrimitive()) {
                JsonElement parsed = GsonCompat.parse(textElement.getAsString());
                if (parsed != null && parsed.isJsonObject()) {
                    return parsed.getAsJsonObject();
                }
            }
            return null;
        }
        return obj;
    }

    private void rememberCommand(String cmdId) {
        if (cmdId == null) {
            return;
        }
        if (recentCmdSet.add(cmdId)) {
            recentCmdIds.addLast(cmdId);
            while (recentCmdIds.size() > MAX_SEEN_CMDS) {
                String evicted = recentCmdIds.removeFirst();
                recentCmdSet.remove(evicted);
            }
        }
    }

    private static String optString(JsonObject obj, String field) {
        if (obj == null || field == null) {
            return null;
        }
        try {
            JsonElement el = obj.get(field);
            if (el == null) {
                return null;
            }
            if (el.isJsonNull()) {
                return null;
            }
            if (el.isJsonPrimitive()) {
                return el.getAsString();
            }
        } catch (Exception ignored) {
        }
        return null;
    }

    private static boolean optBoolean(JsonObject obj, String field, boolean def) {
        try {
            JsonElement el = obj.get(field);
            if (el == null) {
                return def;
            }
            if (el.isJsonPrimitive()) {
                if (el.getAsJsonPrimitive().isBoolean()) {
                    return el.getAsBoolean();
                }
                if (el.getAsJsonPrimitive().isNumber()) {
                    return el.getAsInt() != 0;
                }
                if (el.getAsJsonPrimitive().isString()) {
                    String s = el.getAsString();
                    if ("true".equalsIgnoreCase(s) || "1".equals(s)) {
                        return true;
                    }
                    if ("false".equalsIgnoreCase(s) || "0".equals(s)) {
                        return false;
                    }
                }
            }
        } catch (Exception ignored) {
        }
        return def;
    }

    private static double optDouble(JsonObject obj, String field, double def) {
        try {
            JsonElement el = obj.get(field);
            if (el == null) {
                return def;
            }
            if (el.isJsonPrimitive()) {
                return el.getAsDouble();
            }
        } catch (Exception e) {
            try {
                return Double.parseDouble(obj.get(field).getAsString());
            } catch (Exception ignored) {
            }
        }
        return def;
    }

    private static Double optNullableDouble(JsonObject obj, String field) {
        if (obj == null || field == null || !obj.has(field)) {
            return null;
        }
        try {
            JsonElement el = obj.get(field);
            if (el == null || el.isJsonNull()) {
                return null;
            }
            return el.getAsDouble();
        } catch (Exception e) {
            try {
                return Double.parseDouble(obj.get(field).getAsString());
            } catch (Exception ignored) {
                return null;
            }
        }
    }

    private static long optLong(JsonObject obj, String field, long def) {
        if (obj == null || field == null) {
            return def;
        }
        try {
            JsonElement el = obj.get(field);
            if (el == null) {
                return def;
            }
            if (el.isJsonPrimitive()) {
                if (el.getAsJsonPrimitive().isNumber()) {
                    return el.getAsLong();
                }
                String s = el.getAsString();
                if (s != null && !s.isEmpty()) {
                    return Long.parseLong(s);
                }
            }
        } catch (Exception ignored) {
        }
        return def;
    }
}
