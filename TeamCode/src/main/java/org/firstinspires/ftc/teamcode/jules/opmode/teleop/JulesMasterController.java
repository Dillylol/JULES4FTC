package org.firstinspires.ftc.teamcode.jules.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.google.gson.JsonObject;

import org.firstinspires.ftc.teamcode.jules.core.JulesRobot;

@TeleOp(name = "Jules Master Controller", group = "Jules")
public class JulesMasterController extends OpMode {

    private JulesRobot robot;
    private long lastTelemetryMs = 0;
    private static final long TELEMETRY_PERIOD_MS = 50;

    @Override
    public void init() {
        robot = new JulesRobot(hardwareMap, telemetry);
        robot.init();

        telemetry.addData("JULES", "Master Controller Initialized");
        telemetry.addData("Bridge", robot.bridgeManager != null ? "Ready" : "Offline");

        // Publish Manifest immediately upon init if bridge is ready
        publishManifest();
    }

    @Override
    public void start() {
        // Publish Manifest again on start to be sure
        publishManifest();
    }

    @Override
    public void loop() {
        // 1. Update Robot Systems (Pedro Pathing, etc.)
        robot.update();

        // 2. Process incoming commands from JULES App
        robot.processCommands();

        // 3. Publish Telemetry to JULES App
        long now = System.currentTimeMillis();
        if (now - lastTelemetryMs >= TELEMETRY_PERIOD_MS) {
            JsonObject telem = robot.getTelemetryData();
            telem.addProperty("type", "telemetry");
            telem.addProperty("ts", now);
            robot.publish(telem.toString());
            lastTelemetryMs = now;
        }

        // 4. Update Driver Station Telemetry
        telemetry.addData("Status", "Running");
        if (robot.bridgeManager != null) {
            telemetry.addData("Bridge IP", robot.bridgeManager.getStatusSnapshot().ip);
        }
    }

    @Override
    public void stop() {
        robot.stop();
    }

    private void publishManifest() {
        if (robot.scanner != null) {
            JsonObject manifest = robot.scanner.getManifest();
            manifest.addProperty("type", "manifest");
            manifest.addProperty("ts", System.currentTimeMillis());
            robot.publish(manifest.toString());
            telemetry.addData("JULES", "Manifest Published");
        }
    }
}
