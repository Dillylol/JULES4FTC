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

        StringBuilder sb = new StringBuilder(256);
        sb.append("{\"ts\":").append(m.t)
                .append(",\"cmd\":").append(m.cmdPower)
                .append(",\"vel_ips\":").append(m.velIPS)
                .append(",\"battery_v\":").append(m.batteryV)
                .append(",\"x\":").append(m.x)
                .append(",\"y\":").append(m.y)
                .append(",\"heading\":").append(m.heading)
                .append(",\"heading_deg\":").append(m.headingDeg)
                .append(",\"pitch\":").append(m.pitch)
                .append(",\"roll\":").append(m.roll)
                .append(",\"yawRate\":").append(m.yawRate)
                .append(",\"pitchRate\":").append(m.pitchRate)
                .append(",\"rollRate\":").append(m.rollRate);

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
}