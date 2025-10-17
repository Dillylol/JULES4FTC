package org.firstinspires.ftc.teamcode.jules.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.jules.Metrics;
import org.firstinspires.ftc.teamcode.jules.bridge.JulesCommand;
import org.firstinspires.ftc.teamcode.jules.bridge.JulesBridgeManager;
import org.firstinspires.ftc.teamcode.jules.bridge.JulesMetricsHttpAdapter;
import org.firstinspires.ftc.teamcode.jules.bridge.JulesStreamBus;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.jules.JulesTap;

import java.util.Arrays;
import java.util.List;
import java.util.Locale;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

@TeleOp(name = "JULES: Dev Controller", group = "Jules")
public class JulesDevController extends LinearOpMode {

    // --- Constants for physical properties of the robot ---
    // TODO: Tune these values for your specific robot
    private static final double WHEEL_DIAMETER_INCHES = 3.78; // For standard 96mm goBILDA mecanum wheels
    private static final double GEAR_RATIO = 1.0;            // Assuming no external gearing on the drivetrain
    private static final List<String> COMMAND_REFERENCE = Arrays.asList(
            "DRIVE_FORWARD_<seconds>T_<power>P",
            "DRIVE_BACKWARD_<seconds>T_<power>P",
            "STRAFE_LEFT_<seconds>T_<power>P",
            "STRAFE_RIGHT_<seconds>T_<power>P",
            "TURN_LEFT_<seconds>T_<power>P",
            "TURN_RIGHT_<seconds>T_<power>P",
            "STOP"
    );

    // JULES Components
    private JulesBridgeManager bridgeManager;
    private JulesTap tap;

    // Robot Hardware & Control
    private VoltageSensor vs;
    private IMU imu;
    private Follower follower;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware and JULES components
        setupHardware();
        setupJules();

        JulesBridgeManager.Status status = bridgeManager.getStatusSnapshot();

        telemetry.addLine("✅ JULES Dev Controller Initialized");
        telemetry.addLine(status.advertiseLine);
        telemetry.addLine("\nReady to receive client commands.");
        telemetry.addLine("Supported commands:");
        for (String cmd : COMMAND_REFERENCE) {
            telemetry.addLine(" • " + cmd);
        }
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // The OpMode is now running and waiting for commands.
        while (opModeIsActive() && !isStopRequested()) {
            // CRITICAL: follower.update() must be called every loop to update odometry
            follower.update();

            // Refresh stream bus reference in case the bridge toggled.
            JulesStreamBus streamBus = bridgeManager.getStreamBus();

            // Get the latest command from the client. This also clears it.
            String command = JulesCommand.getAndClearCommand();

            if (command != null) {
                telemetry.addData("Received Command", command);
                telemetry.update();

                parseAndExecute(command, streamBus); // Execute the command

                // Ensure the robot is stopped after the command finishes
                follower.setTeleOpDrive(0, 0, 0, true);
            } else {
                telemetry.addData("Status", "IDLE - Waiting for command.");
                telemetry.update();
            }

            sleep(50); // Poll for commands at 20Hz
        }
    }

    /**
     * Parses the command string and calls the appropriate robot action.
     * @param command The command string from the client (e.g., "DRIVE_FORWARD_1.5T_0.5P")
     * @param streamBus Optional live stream bus used to mirror command execution to clients.
     */
    private void parseAndExecute(String command, JulesStreamBus streamBus) {
        if (command == null) {
            return;
        }

        String sanitized = command.trim();
        if (sanitized.isEmpty()) {
            return;
        }

        String upper = sanitized.toUpperCase(Locale.US);
     */
    private void parseAndExecute(String commandRaw) {
        if (commandRaw == null) {
            return;
        }

        String command = commandRaw.trim();
        if (command.isEmpty()) {
            return;
        }

        String upper = command.toUpperCase(Locale.US);

        double duration = Math.max(0, parseValue(upper, "T", 1.0));
        double power = clampPower(parseValue(upper, "P", 0.5));

        double magnitude = Math.abs(power);

        if (upper.startsWith("DRIVE_FORWARD")) {
            executeMovement(duration, magnitude, 0, 0, sanitized, streamBus);
        } else if (upper.startsWith("DRIVE_BACKWARD")) {
            executeMovement(duration, -magnitude, 0, 0, sanitized, streamBus);
        } else if (upper.startsWith("STRAFE_LEFT")) {
            executeMovement(duration, 0, -magnitude, 0, sanitized, streamBus);
        } else if (upper.startsWith("STRAFE_RIGHT")) {
            executeMovement(duration, 0, magnitude, 0, sanitized, streamBus);
        } else if (upper.startsWith("TURN_LEFT")) {
            executeMovement(duration, 0, 0, -magnitude, sanitized, streamBus);
        } else if (upper.startsWith("TURN_RIGHT")) {
            executeMovement(duration, 0, 0, magnitude, sanitized, streamBus);
            executeMovement(duration, magnitude, 0, 0, command);
        } else if (upper.startsWith("DRIVE_BACKWARD")) {
            executeMovement(duration, -magnitude, 0, 0, command);
        } else if (upper.startsWith("STRAFE_LEFT")) {
            executeMovement(duration, 0, -magnitude, 0, command);
        } else if (upper.startsWith("STRAFE_RIGHT")) {
            executeMovement(duration, 0, magnitude, 0, command);
        } else if (upper.startsWith("TURN_LEFT")) {
            executeMovement(duration, 0, 0, -magnitude, command);
        } else if (upper.startsWith("TURN_RIGHT")) {
            executeMovement(duration, 0, 0, magnitude, command);
        } else if (upper.startsWith("STOP")) {
            follower.setTeleOpDrive(0, 0, 0, true);
            telemetry.addData("Command", "STOP");
            telemetry.update();
        } else {
            telemetry.addData("Unknown Command", sanitized);
            telemetry.addData("Unknown Command", command);
            telemetry.update();
        }
    }

    /**
     * Executes a teleop-style movement for a specified duration while streaming data.
     */
    private void executeMovement(double duration, double y, double x, double r, String label, JulesStreamBus bus) {
        ElapsedTime timer = new ElapsedTime();
        double lastEmit = 0;
        final double PERIOD_S = 0.02; // 50 Hz data streaming rate

        boolean startLabelSent = false;
        if (bus != null) {
            bus.publishJsonLine(String.format("{\"label\":\"START: %s\"}", label));
            startLabelSent = true;
        }

        while (opModeIsActive() && timer.seconds() < duration) {
            // Keep odometry and robot state updated
            follower.update();

            // Command the robot using Pedro Pathing's teleop drive
            follower.setTeleOpDrive(y, x, r, true);

            // Set the command power for logging purposes
            double commandPower = y != 0 ? y : (x != 0 ? x : r);
            tap.setLastCmd(commandPower);

            // Stream data at the specified frequency (50Hz)
            double now = getRuntime();
            if (now - lastEmit >= PERIOD_S) {
                lastEmit = now;
                Metrics m = tap.sample(imu, follower);
                JulesStreamBus currentBus = bus != null ? bus : bridgeManager.getStreamBus();
                if (currentBus != null) {
                    if (!startLabelSent) {
                        currentBus.publishJsonLine(String.format("{\"label\":\"START: %s\"}", label));
                        startLabelSent = true;
                    }
                    currentBus.publishJsonLine(JulesMetricsHttpAdapter.encodePublic(m));
                    bus = currentBus;
                }
            }

            telemetry.addData("Executing", "%s", label);
            telemetry.addData("Time", "%.1f / %.1f sec", timer.seconds(), duration);
            telemetry.update();
        }

        // Add a label to mark the end of the test
        JulesStreamBus endBus = bus != null ? bus : bridgeManager.getStreamBus();
        if (startLabelSent && endBus != null) {
            endBus.publishJsonLine(String.format("{\"label\":\"END: %s\"}", label));
        }
    }

    /**
     * A helper to parse numeric values from a command string.
     */
    private double parseValue(String command, String id, double defaultValue) {
        Pattern p = Pattern.compile("([-+]?\\d*\\.?\\d+)" + id);
        Matcher m = p.matcher(command);
        if (m.find()) {
            try {
                return Double.parseDouble(m.group(1));
            } catch (NumberFormatException e) {
                return defaultValue;
            }
        }
        return defaultValue;
    }

    private double clampPower(double power) {
        if (Double.isNaN(power)) {
            return 0;
        }
        return Math.max(-1.0, Math.min(1.0, power));
    }

    // --- Helper methods for initialization ---

    private void setupJules() {
        bridgeManager = JulesBridgeManager.getInstance();
        bridgeManager.prepare(hardwareMap.appContext);
    }

    private void setupHardware() {
        // Initialize Pedro Pathing follower
        follower = Constants.createFollower(hardwareMap);

        // Get hardware references
        imu = hardwareMap.get(IMU.class, "imu");
        vs = hardwareMap.voltageSensor.iterator().next();

        // Create the JULES data tap with your robot's specific motors and constants
        List<DcMotorEx> motors = Arrays.asList(
                hardwareMap.get(DcMotorEx.class, "lf"),
                hardwareMap.get(DcMotorEx.class, "rf"),
                hardwareMap.get(DcMotorEx.class, "lr"),
                hardwareMap.get(DcMotorEx.class, "rr")
        );

        tap = new JulesTap(
                537.7, // Ticks Per Revolution for your motors (e.g., goBILDA 5203 series)
                WHEEL_DIAMETER_INCHES,
                GEAR_RATIO,
                vs,
                motors);
    }
}