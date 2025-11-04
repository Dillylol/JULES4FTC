package org.firstinspires.ftc.teamcode.jules.shot;

import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;

import org.firstinspires.ftc.teamcode.jules.bridge.JulesStreamBus;

/**
 * Handles all messaging with the off-board shot planner. Outbound helpers
 * publish JSON frames onto the {@link JulesStreamBus}, inbound helpers parse the
 * mailbox payloads into {@link ShotPlan} objects or RPM model updates.
 */
public final class ShotPlannerBridge {
    private final JulesStreamBus streamBus;

    public ShotPlannerBridge(JulesStreamBus streamBus) {
        this.streamBus = streamBus;
    }

    public void sendRequestShotPlan(double distanceIn,
                                    double vBatt,
                                    PedroShotNavigator.PoseSnapshot pose,
                                    double rpmHint) {
        if (streamBus == null) {
            return;
        }
        JsonObject root = new JsonObject();
        root.addProperty("type", "request_shot_plan");
        root.addProperty("distance_in", distanceIn);
        root.addProperty("v_batt_load", vBatt);
        root.addProperty("rpm_hint", rpmHint);

        JsonObject poseObj = new JsonObject();
        poseObj.addProperty("x_in", pose != null ? pose.xIn : 0.0);
        poseObj.addProperty("y_in", pose != null ? pose.yIn : 0.0);
        poseObj.addProperty("heading_deg", pose != null ? pose.headingDeg : 0.0);
        root.add("pose", poseObj);

        publish(root);
    }

    public void sendObsShot(double distanceIn,
                             double vBatt,
                             int rpmTarget,
                             int rpmAtFire,
                             int hit,
                             long latencyMs,
                             PedroShotNavigator.PoseSnapshot pose,
                             double headingToTagDeg) {
        if (streamBus == null) {
            return;
        }
        JsonObject root = new JsonObject();
        root.addProperty("type", "obs_shot");
        root.addProperty("distance_in", distanceIn);
        root.addProperty("v_batt_load", vBatt);
        root.addProperty("rpm_tgt", rpmTarget);
        root.addProperty("rpm_at_fire", rpmAtFire);
        root.addProperty("hit", hit);
        if (latencyMs >= 0) {
            root.addProperty("latency_ms", latencyMs);
        }
        root.addProperty("pose_x", pose != null ? pose.xIn : 0.0);
        root.addProperty("pose_y", pose != null ? pose.yIn : 0.0);
        root.addProperty("heading_to_tag", headingToTagDeg);
        publish(root);
    }

    public String unwrapCommandEnvelope(String raw) {
        if (raw == null) {
            return null;
        }
        String candidate = raw.trim();
        for (int depth = 0; depth < 3; depth++) {
            JsonElement parsed = parse(candidate);
            if (parsed == null || !parsed.isJsonObject()) {
                break;
            }
            JsonObject obj = parsed.getAsJsonObject();
            String type = optString(obj, "type");
            if (type != null && type.equalsIgnoreCase("cmd") && obj.has("text")) {
                JsonElement text = obj.get("text");
                if (text.isJsonObject()) {
                    candidate = text.getAsJsonObject().toString();
                    continue;
                }
                if (text.isJsonPrimitive()) {
                    String next = text.getAsJsonPrimitive().getAsString();
                    if (next != null && !next.isEmpty()) {
                        candidate = next.trim();
                        continue;
                    }
                }
            }
            break;
        }
        return candidate;
    }

    public ShotPlan tryParseShotPlan(String raw) {
        JsonObject payload = extractPayload(raw, "shot_plan");
        if (payload == null) {
            return null;
        }
        double bias = optDouble(payload, "rpm_bias", 0.0);
        boolean fireNow = optBoolean(payload, "fire_now", false);
        boolean loiter = optBoolean(payload, "loiter", false);
        ShotPlan plan = new ShotPlan(bias, fireNow, loiter);
        plan.setReceivedAtMs(System.currentTimeMillis());
        return plan;
    }

    public JsonObject tryParseRpmModelUpdate(String raw) {
        return extractPayload(raw, "rpm_model_update");
    }

    private JsonObject extractPayload(String raw, String typeKey) {
        if (raw == null || raw.isEmpty()) {
            return null;
        }
        JsonElement parsed = parse(raw);
        if (parsed == null || !parsed.isJsonObject()) {
            return null;
        }
        JsonObject obj = parsed.getAsJsonObject();
        if (obj.has(typeKey) && obj.get(typeKey).isJsonObject()) {
            return obj.getAsJsonObject(typeKey);
        }
        String type = optString(obj, "type");
        if (type != null && type.equalsIgnoreCase(typeKey)) {
            return obj;
        }
        return null;
    }

    private void publish(JsonObject json) {
        try {
            streamBus.publishJsonLine(json.toString());
        } catch (Exception ignored) {
        }
    }

    private static JsonElement parse(String raw) {
        try {
            return JsonParser.parseString(raw);
        } catch (Exception ignored) {
            return null;
        }
    }

    private static String optString(JsonObject obj, String key) {
        if (obj == null || key == null) {
            return null;
        }
        if (obj.has(key) && obj.get(key).isJsonPrimitive()) {
            try {
                return obj.get(key).getAsString();
            } catch (Exception ignored) {
                return null;
            }
        }
        return null;
    }

    private static boolean optBoolean(JsonObject obj, String key, boolean def) {
        if (obj != null && obj.has(key) && obj.get(key).isJsonPrimitive()) {
            try {
                return obj.get(key).getAsBoolean();
            } catch (Exception ignored) {
                return def;
            }
        }
        return def;
    }

    private static double optDouble(JsonObject obj, String key, double def) {
        if (obj != null && obj.has(key) && obj.get(key).isJsonPrimitive()) {
            try {
                return obj.get(key).getAsDouble();
            } catch (Exception ignored) {
                return def;
            }
        }
        return def;
    }
}
