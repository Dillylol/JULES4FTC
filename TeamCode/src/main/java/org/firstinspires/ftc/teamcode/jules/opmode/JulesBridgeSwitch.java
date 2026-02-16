package org.firstinspires.ftc.teamcode.jules.opmode;

import android.content.Context;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.jules.bridge.JulesBridgeManager;

/**
 * Simple toggle OpMode that starts/stops the persistent JULES bridge.
 * Run this once to enable the bridge — it stays running across OpModes.
 */
@TeleOp(name = "JULES: Enable & Status", group = "Jules")
public class JulesBridgeSwitch extends OpMode {

    private JulesBridgeManager manager;

    @Override
    public void init() {
        manager = JulesBridgeManager.getInstance();
        Context appContext = hardwareMap.appContext;
        manager.prepare(appContext);

        // Inject Battery Sensor so heartbeat includes real voltage
        try {
            com.qualcomm.robotcore.hardware.VoltageSensor vs = hardwareMap.voltageSensor.iterator().next();
            manager.setBatterySensor(vs);
        } catch (Exception ignored) {
        }

        telemetry.addLine("Press START to enable the JULES bridge.");
        telemetry.addData("IP", manager.defaultIp());
        telemetry.addData("Port", manager.port());
        telemetry.update();
    }

    @Override
    public void start() {
        manager.start(manager.defaultIp(), "");
        manager.setAutoEnabled(true);

        telemetry.addLine("Bridge enabled. Leave this OpMode; bridge remains ON.");
        telemetry.update();
    }

    @Override
    public void loop() {
        JulesBridgeManager.Status status = manager.getStatusSnapshot();
        renderTelemetry(status);
    }

    @Override
    public void stop() {
        telemetry.addLine("Bridge remains ON.");
        telemetry.update();
    }

    private static String nullSafe(String s) {
        return s == null ? "-" : s;
    }

    private String formatDuration(long ms) {
        long totalSeconds = ms / 1000L;
        long hours = totalSeconds / 3600L;
        long minutes = (totalSeconds % 3600L) / 60L;
        long seconds = totalSeconds % 60L;
        if (hours > 0)
            return String.format("%dh %02dm %02ds", hours, minutes, seconds);
        if (minutes > 0)
            return String.format("%dm %02ds", minutes, seconds);
        return String.format("%ds", seconds);
    }

    private void renderTelemetry(JulesBridgeManager.Status status) {
        String state = (status != null && status.running) ? "RUNNING" : "STOPPED";
        long uptime = (status != null) ? status.uptimeMs : 0L;
        String ip = (status != null) ? status.ip : manager.defaultIp();
        int port = (status != null) ? status.port : manager.port();
        String wsUrl = (status != null) ? nullSafe(status.wsUrl) : nullSafe(manager.getWsUrl());
        int retries = (status != null) ? status.retryCount : 0;
        String lastError = (status != null) ? status.lastError : null;

        telemetry.addData("Status", state);
        telemetry.addData("IP", nullSafe(ip));
        telemetry.addData("Port", port);
        telemetry.addData("WS URL", wsUrl);
        telemetry.addData("Uptime", formatDuration(uptime));
        telemetry.addData("Retries", retries);
        if (lastError != null && !lastError.isEmpty()) {
            telemetry.addData("Last error", lastError);
        }
        telemetry.addLine("Bridge remains ON across OpModes.");
        telemetry.update();
    }
}