package org.firstinspires.ftc.teamcode.jules.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.jules.JulesTap;
import org.firstinspires.ftc.teamcode.jules.bridge.JulesBridgeManager;
import org.firstinspires.ftc.teamcode.jules.bridge.JulesCommand;
import org.firstinspires.ftc.teamcode.jules.bridge.JulesStreamBus;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * SIMPLE dev controller: drivetrain only, no odometry/follower.
 * Consumes the persistent JULES bridge and executes basic commands.
 *
 * Supported (case-insensitive):
 *  DRIVE_FORWARD_<seconds>T_<power>P
 *  DRIVE_BACKWARD_<seconds>T_<power>P
 *  STRAFE_LEFT_<seconds>T_<power>P
 *  STRAFE_RIGHT_<seconds>T_<power>P
 *  TURN_LEFT_<seconds>T_<power>P
 *  TURN_RIGHT_<seconds>T_<power>P
 *  STOP
 */
@TeleOp(name = "JULES: Dev Controller (Simple)", group = "Jules")
public class JulesDevController extends LinearOpMode {

    // Bridge
    private JulesBridgeManager bridgeManager;

    // Drivetrain motors
    private DcMotorEx lf, rf, lr, rr;

    // Optional sensors for metrics (kept but not required)
    private IMU imu;
    private JulesTap tap; // used only for simple power sampling; safe to keep

    @Override
    public void runOpMode() throws InterruptedException {
        setupHardware();
        setupJules();

        telemetry.addLine("✅ JULES Dev Controller (Simple) Initialized");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            JulesStreamBus bus = (bridgeManager != null) ? bridgeManager.getStreamBus() : null;

            String raw = JulesCommand.getAndClearCommand();
            if (raw != null) {
                telemetry.addData("Received", raw);
                telemetry.update();
                parseAndExecute(raw, bus);
                // Always coast to stop after command completes
                setDrive(0, 0, 0);
            } else {
                telemetry.addData("Status", "IDLE - Waiting for command.");
                telemetry.update();
            }

            sleep(50); // ~20 Hz
        }
    }

    // --- Command parsing ---
    private void parseAndExecute(String commandRaw, JulesStreamBus bus) {
        if (commandRaw == null) return;
        String command = commandRaw.trim();
        if (command.isEmpty()) return;

        String upper = command.toUpperCase(Locale.US);
        double duration = Math.max(0, parseValue(upper, "T", 1.0));
        double power = clampPower(parseValue(upper, "P", 0.5));
        double mag = Math.abs(power);

        if (upper.startsWith("DRIVE_FORWARD")) {
            executeMovement(duration, mag, 0, 0, command, bus);
        } else if (upper.startsWith("DRIVE_BACKWARD")) {
            executeMovement(duration, -mag, 0, 0, command, bus);
        } else if (upper.startsWith("STRAFE_LEFT")) {
            executeMovement(duration, 0, -mag, 0, command, bus);
        } else if (upper.startsWith("STRAFE_RIGHT")) {
            executeMovement(duration, 0, mag, 0, command, bus);
        } else if (upper.startsWith("TURN_LEFT")) {
            executeMovement(duration, 0, 0, -mag, command, bus);
        } else if (upper.startsWith("TURN_RIGHT")) {
            executeMovement(duration, 0, 0, mag, command, bus);
        } else if (upper.startsWith("STOP")) {
            setDrive(0, 0, 0);
            if (tap != null) tap.setLastCmd(0);
            if (bus != null) bus.publishJsonLine("{\"label\":\"STOP\"}");
            telemetry.addData("Command", "STOP");
            telemetry.update();
        } else {
            telemetry.addData("Unknown", command);
            telemetry.update();
        }
    }

    // --- Movement execution (no follower, just timed power) ---
    private void executeMovement(double durationSec,
                                 double y, double x, double r,
                                 String label,
                                 JulesStreamBus bus) {
        ElapsedTime timer = new ElapsedTime();
        double lastEmit = 0;
        final double PERIOD_S = 0.02; // 50 Hz optional stream

        boolean startLabelSent = false;
        if (bus != null) {
            bus.publishJsonLine(String.format("{\"label\":\"START: %s\"}", label));
            startLabelSent = true;
        }

        while (opModeIsActive() && timer.seconds() < durationSec) {
            setDrive(y, x, r);

            double commandPower = (y != 0) ? y : (x != 0 ? x : r);
            if (tap != null) tap.setLastCmd(commandPower);

            double now = getRuntime();
            if (now - lastEmit >= PERIOD_S) {
                lastEmit = now;
                JulesStreamBus currentBus = (bus != null) ? bus : (bridgeManager != null ? bridgeManager.getStreamBus() : null);
                if (currentBus != null) {
                    if (!startLabelSent) {
                        currentBus.publishJsonLine(String.format("{\"label\":\"START: %s\"}", label));
                        startLabelSent = true;
                    }
                    // For the simple version we only stream a compact status line
                    currentBus.publishJsonLine(String.format("{\"drive\":%.3f,\"strafe\":%.3f,\"turn\":%.3f}", y, x, r));
                    bus = currentBus;
                }
            }

            telemetry.addData("Executing", label);
            telemetry.addData("Time", "%.1f / %.1f s", timer.seconds(), durationSec);
            telemetry.update();
        }

        JulesStreamBus endBus = (bus != null) ? bus : (bridgeManager != null ? bridgeManager.getStreamBus() : null);
        if (startLabelSent && endBus != null) {
            endBus.publishJsonLine(String.format("{\"label\":\"END: %s\"}", label));
        }

        setDrive(0, 0, 0);
        if (tap != null) tap.setLastCmd(0);
    }

    // --- Drivetrain helper (mecanum standard power mix) ---
    private void setDrive(double y, double x, double r) {
        double fl = y + x + r;
        double bl = y - x + r;
        double fr = y - x - r;
        double br = y + x - r;
        double max = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));
        lf.setPower(fl / max); rf.setPower(fr / max);
        lr.setPower(bl / max); rr.setPower(br / max);
    }

    // --- Utils ---
    private double parseValue(String command, String id, double def) {
        Pattern p = Pattern.compile("([-+]?\\d*\\.?\\d+)" + id);
        Matcher m = p.matcher(command);
        if (m.find()) { try { return Double.parseDouble(m.group(1)); } catch (NumberFormatException ignored) {} }
        return def;
    }

    private double clampPower(double p) { return (Double.isNaN(p)) ? 0 : Math.max(-1.0, Math.min(1.0, p)); }

    private void setupJules() { bridgeManager = JulesBridgeManager.getInstance(); }

    private void setupHardware() {
        lf = hardwareMap.get(DcMotorEx.class, "lf");
        rf = hardwareMap.get(DcMotorEx.class, "rf");
        lr = hardwareMap.get(DcMotorEx.class, "lr");
        rr = hardwareMap.get(DcMotorEx.class, "rr");

        // Optional sensors if you still want to sample power/battery later
        try { imu = hardwareMap.get(IMU.class, "imu"); } catch (Exception ignored) {}
        List<VoltageSensor> voltageSensors = new ArrayList<>();
        try {
            for (VoltageSensor sensor : hardwareMap.voltageSensor) {
                if (sensor != null) {
                    voltageSensors.add(sensor);
                }
            }
        } catch (Exception ignored) {}

        List<DcMotorEx> motors = Arrays.asList(lf, rf, lr, rr);
        tap = voltageSensors.isEmpty()
                ? new JulesTap(537.7, /*TPR example*/ 3.78, /*wheel D in*/ 1.0, (VoltageSensor) null, motors)
                : new JulesTap(537.7, /*TPR example*/ 3.78, /*wheel D in*/ 1.0, voltageSensors, motors);
    }
}
