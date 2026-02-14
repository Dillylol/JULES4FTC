package org.firstinspires.ftc.teamcode.jules.opmode;

import android.content.Context;

import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.jules.bridge.JulesBridgeManager;

/**
 * Panels-style toggle OpMode that starts/stops the persistent JULES bridge
 * and mirrors status to BOTH the FTC Driver Station telemetry and the Panels UI telemetry.
 *
 * Panels will display the **full (unmasked) token** for fast pairing.
 * Driver Station will continue to display the masked token.
 */
@TeleOp(name = "JULES: Enable & Status", group = "Jules")
public class JulesBridgeSwitch extends OpMode {

    private JulesBridgeManager manager;
    private String preparedIp;
    private String preparedToken;

    // Panels telemetry (separate from FTC DS telemetry)
    private Telemetry panelsTl;

    @Override
    public void init() {
        manager = JulesBridgeManager.getInstance();
        Context appContext = hardwareMap.appContext;
        manager.prepare(appContext);
        preparedIp = manager.defaultIp();
        preparedToken = manager.ensureToken(appContext);

        panelsTl = PanelsTelemetry.INSTANCE.getFtcTelemetry();

        telemetry.addLine("Press START to enable the persistent JULES bridge.");
        telemetry.addData("IP", preparedIp);
        telemetry.addData("Token", JulesBridgeManager.maskToken(preparedToken));
        telemetry.update();

        if (panelsTl != null) {
            panelsTl.addLine("Press START to enable the persistent JULES bridge.");
            panelsTl.addData("IP", preparedIp);
            panelsTl.addData("Token", preparedToken);
            panelsTl.update();
        }
    }

    @Override
    public void start() {
        manager.start(preparedIp, preparedToken);
        manager.setAutoEnabled(true);

        telemetry.addLine("Bridge enable requested. Leave this OpMode; bridge remains ON.");
        telemetry.update();

        if (panelsTl != null) {
            panelsTl.addLine("Bridge enable requested. Leave this OpMode; bridge remains ON.");
            panelsTl.update();
        }
    }

    @Override
    public void loop() {
        JulesBridgeManager.Status status = manager.getStatusSnapshot();

        renderTelemetry(status);
    }

    @Override
    public void stop() {
        telemetry.addLine("Leave this OpMode; bridge remains ON.");
        telemetry.update();

        if (panelsTl != null) {
            panelsTl.addLine("Leave this OpMode; bridge remains ON.");
            panelsTl.update();
        }
    }

    private static String nullSafe(String s) { return s == null ? "-" : s; }

    private String formatDuration(long ms) {
        long totalSeconds = ms / 1000L;
        long hours = totalSeconds / 3600L;
        long minutes = (totalSeconds % 3600L) / 60L;
        long seconds = totalSeconds % 60L;
        if (hours > 0) return String.format("%dh %02dm %02ds", hours, minutes, seconds);
        if (minutes > 0) return String.format("%dm %02ds", minutes, seconds);
        return String.format("%ds", seconds);
    }

    private void renderTelemetry(JulesBridgeManager.Status status) {
        String state = (status != null && status.running) ? "RUNNING" : "STOPPED";
        long uptime = (status != null) ? status.uptimeMs : 0L;
        String ip = (status != null) ? status.ip : manager.defaultIp();
        int port = (status != null) ? status.port : manager.port();
        String tokenMasked = (status != null) ? JulesBridgeManager.maskToken(status.token) : JulesBridgeManager.maskToken(preparedToken);
        String tokenFull = (status != null && status.token != null) ? status.token : preparedToken;
        int retries = (status != null) ? status.retryCount : 0;
        String lastError = (status != null) ? status.lastError : null;
        String advertise = (status != null) ? nullSafe(status.advertiseLine) : nullSafe(manager.getAdvertiseLine());
        String wsUrl = (status != null) ? nullSafe(status.wsUrl) : nullSafe(manager.getWsUrl());

        telemetry.addData("Status", state);
        telemetry.addData("IP", nullSafe(ip));
        telemetry.addData("Port", port);
        telemetry.addData("WS URL", wsUrl);
        telemetry.addData("Token", tokenMasked);
        telemetry.addData("Uptime", formatDuration(uptime));
        telemetry.addData("Retries", retries);
        if (lastError != null && !lastError.isEmpty()) {
            telemetry.addData("Last error", lastError);
        }
        telemetry.addLine(advertise);
        telemetry.addLine("Leave this OpMode; bridge remains ON.");
        telemetry.update();

        if (panelsTl != null) {
            panelsTl.addData("Status", state);
            panelsTl.addData("IP", nullSafe(ip));
            panelsTl.addData("Port", port);
            panelsTl.addData("WS URL", wsUrl);
            panelsTl.addData("Token", nullSafe(tokenFull));
            panelsTl.addData("Uptime", formatDuration(uptime));
            panelsTl.addData("Retries", retries);
            if (lastError != null && !lastError.isEmpty()) {
                panelsTl.addData("Last error", lastError);
            }
            panelsTl.addLine(advertise);
            panelsTl.addLine("Leave this OpMode; bridge remains ON.");
            panelsTl.update();
        }
    }
}