package org.firstinspires.ftc.teamcode.jules.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.jules.bridge.JulesBridgeManager;

/**
 * Panels-style toggle OpMode that starts/stops the persistent JULES bridge.
 */
@TeleOp(name = "JULES: Bridge Switch", group = "Jules")
public class JulesBridgeSwitch extends OpMode {

    private JulesBridgeManager manager;
    private String preparedIp;
    private String preparedToken;

    @Override
    public void init() {
        manager = JulesBridgeManager.getInstance();
        manager.prepare(hardwareMap.appContext);
        preparedIp = manager.defaultIp();
        preparedToken = manager.ensureToken(hardwareMap.appContext);

        telemetry.addLine("Press START to enable JULES");
        telemetry.addData("IP", preparedIp);
        telemetry.addData("Token", JulesBridgeManager.maskToken(preparedToken));
        telemetry.update();
    }

    @Override
    public void start() {
        manager.start(preparedIp, preparedToken);
    }

    @Override
    public void loop() {
        JulesBridgeManager.Status status = manager.getStatusSnapshot();
        telemetry.addData("Status", status.running ? "RUNNING" : "STOPPED");
        telemetry.addData("IP", status.ip);
        telemetry.addData("Port", status.port);
        telemetry.addData("Token", status.maskedToken);
        telemetry.addData("Uptime", formatDuration(status.uptimeMs));
        telemetry.addData("Retries", status.retryCount);
        if (status.lastError != null && !status.lastError.isEmpty()) {
            telemetry.addData("Last error", status.lastError);
        }
        telemetry.addLine(status.advertiseLine);
        if (!status.running) {
            telemetry.addLine("Press START to enable JULES");
        }
        telemetry.update();
    }

    @Override
    public void stop() {
        manager.stop();
    }

    private String formatDuration(long ms) {
        long totalSeconds = ms / 1000L;
        long hours = totalSeconds / 3600L;
        long minutes = (totalSeconds % 3600L) / 60L;
        long seconds = totalSeconds % 60L;
        if (hours > 0) {
            return String.format("%dh %02dm %02ds", hours, minutes, seconds);
        }
        if (minutes > 0) {
            return String.format("%dm %02ds", minutes, seconds);
        }
        return String.format("%ds", seconds);
    }
}
