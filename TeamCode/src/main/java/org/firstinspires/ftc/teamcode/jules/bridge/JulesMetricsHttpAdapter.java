// File: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/jules/bridge/JulesMetricsHttpAdapter.java
package org.firstinspires.ftc.teamcode.jules.bridge;

import org.firstinspires.ftc.teamcode.jules.Metrics;

public class JulesMetricsHttpAdapter {
    private final JulesBuffer buffer;

    public JulesMetricsHttpAdapter(JulesBuffer buffer) {
        this.buffer = buffer;
    }

    public static String encodePublic(Metrics m) {
        if (m == null) return "{}";

        double tsSeconds = m.t;
        long tsMillis = (long) Math.round(tsSeconds * 1000.0);

        StringBuilder sb = new StringBuilder(256);
        sb.append("{\"type\":\"metrics\"");
        appendNumber(sb, "ts_s", tsSeconds);
        sb.append(",\"ts_ms\":").append(tsMillis);
        appendNumber(sb, "cmd", m.cmdPower);
        appendNumber(sb, "vel_ips", m.velIPS);
        appendNumber(sb, "battery_v", m.batteryV);
        appendNumber(sb, "x", m.x);
        appendNumber(sb, "y", m.y);
        appendNumber(sb, "heading", m.heading);
        appendNumber(sb, "heading_deg", m.headingDeg);
        appendNumber(sb, "pitch", m.pitch);
        appendNumber(sb, "roll", m.roll);
        appendNumber(sb, "yawRate", m.yawRate);
        appendNumber(sb, "pitchRate", m.pitchRate);
        appendNumber(sb, "rollRate", m.rollRate);

        // --- CORRECTED SECTION ---
        // If a label exists, escape special characters and add it to the JSON.
        if (m.label != null && !m.label.isEmpty()) {
            String safeLabel = m.label.replace("\\", "\\\\").replace("\"", "\\\"");
            sb.append(",\"label\":\"").append(safeLabel).append("\"");
        }
        if (m.jsonData != null && !m.jsonData.isEmpty()) {
            sb.append(",\"data\":").append(m.jsonData);
        }
        // -------------------------

        sb.append("}");
        return sb.toString();
    }

    private static void appendNumber(StringBuilder sb, String key, double value) {
        sb.append(",\"").append(key).append("\":");
        if (Double.isFinite(value)) {
            sb.append(value);
        } else {
            sb.append("null");
        }
    }
}