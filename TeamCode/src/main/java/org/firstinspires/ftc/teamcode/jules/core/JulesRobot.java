package org.firstinspires.ftc.teamcode.jules.core;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.jules.bridge.JulesBridgeManager;
import org.firstinspires.ftc.teamcode.jules.bridge.JulesStreamBus;
import org.firstinspires.ftc.teamcode.jules.constants.JulesConstants;

/**
 * JULES Robot Core Abstraction (Agnostic).
 * Handles hardware mapping using JulesConstants and JULES Bridge communication.
 */
public class JulesRobot {
    private static final String TAG = "JulesRobot";

    public final HardwareMap hardwareMap;
    public final Telemetry telemetry;

    // Hardware
    public DcMotor leftFront, leftRear, rightFront, rightRear;
    public VoltageSensor batterySensor;
    public IMU imu;
    public Follower follower; // Pedro Pathing

    // JULES Bridge
    public JulesBridgeManager bridgeManager;
    public JulesStreamBus streamBus;

    // Dynamic Hardware Scanner
    public final JulesHardwareScanner scanner = new JulesHardwareScanner();

    private JulesStreamBus.Subscription subscription;

    public JulesRobot(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }

    public void init() {
        initHardware();
        initBridge();
        // Dynamic scan
        scanner.scan(hardwareMap);

        // Advertise manifest if bridge is ready
        if (bridgeManager != null) {
            bridgeManager.setHardwareScanner(scanner);
        }
    }

    public void stop() {
        if (subscription != null) {
            subscription.close();
            subscription = null;
        }
    }

    private void initHardware() {
        try {
            leftFront = hardwareMap.tryGet(DcMotor.class, JulesConstants.Motors.FRONT_LEFT);
            leftRear = hardwareMap.tryGet(DcMotor.class, JulesConstants.Motors.BACK_LEFT);
            rightFront = hardwareMap.tryGet(DcMotor.class, JulesConstants.Motors.FRONT_RIGHT);
            rightRear = hardwareMap.tryGet(DcMotor.class, JulesConstants.Motors.BACK_RIGHT);

            if (leftFront != null)
                leftFront.setDirection(JulesConstants.Motors.LEFT_DIR);
            if (leftRear != null)
                leftRear.setDirection(JulesConstants.Motors.LEFT_DIR);
            if (rightFront != null)
                rightFront.setDirection(JulesConstants.Motors.RIGHT_DIR);
            if (rightRear != null)
                rightRear.setDirection(JulesConstants.Motors.RIGHT_DIR);

            // Battery compensation
            for (VoltageSensor sensor : hardwareMap.getAll(VoltageSensor.class)) {
                batterySensor = sensor;
                break;
            }

            // IMU Initialize
            if (JulesConstants.Sensors.IMU != null) {
                imu = hardwareMap.tryGet(IMU.class, JulesConstants.Sensors.IMU);
                if (imu != null) {
                    IMU.Parameters parameters = new IMU.Parameters(
                            new RevHubOrientationOnRobot(
                                    JulesConstants.IMU.LOGO_DIR,
                                    JulesConstants.IMU.USB_DIR));
                    imu.initialize(parameters);
                }
            }

            // Pedro Pathing Initialize
            if (JulesConstants.USE_PEDRO_PATHING) {
                try {
                    follower = org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower(hardwareMap);
                } catch (Exception e) {
                    RobotLog.e(TAG, "Pedro Pathing Init Failed: " + e.getMessage());
                }
            }

        } catch (Exception e) {
            telemetry.addData("JulesRobot", "Hardware mapping error: " + e.getMessage());
            RobotLog.w(TAG, "Hardware Init Warning: " + e.getMessage());
        }
    }

    private void initBridge() {
        try {
            bridgeManager = JulesBridgeManager.getInstance();
            if (bridgeManager != null) {
                bridgeManager.prepare(hardwareMap.appContext);
                streamBus = bridgeManager.getStreamBus();
                if (streamBus != null) {
                    subscription = streamBus.subscribe();
                }
            }
        } catch (Exception e) {
            RobotLog.e(TAG, "Bridge Init Failed", e);
        }
    }

    public void setDrivePowers(double lf, double lr, double rf, double rr) {
        if (JulesConstants.USE_BATTERY_OPTIMIZATION && batterySensor != null) {
            double voltage = batterySensor.getVoltage();
            double scale = Math.min(1.0, 12.0 / voltage); // Scale against 12V
            lf *= scale;
            lr *= scale;
            rf *= scale;
            rr *= scale;
        }

        if (leftFront != null)
            leftFront.setPower(lf);
        if (leftRear != null)
            leftRear.setPower(lr);
        if (rightFront != null)
            rightFront.setPower(rf);
        if (rightRear != null)
            rightRear.setPower(rr);
    }

    public void publish(String json) {
        if (streamBus != null) {
            streamBus.publishJsonLine(json);
        }
    }

    /**
     * Poll and execute commands from the JULES Bridge.
     * Call this in your OpMode loop().
     */
    public void processCommands() {
        if (subscription == null)
            return;

        String line;
        while ((line = subscription.poll()) != null) {
            try {
                com.google.gson.JsonObject event = org.firstinspires.ftc.teamcode.jules.bridge.util.GsonCompat
                        .parse(line).getAsJsonObject();
                if (event.has("type") && "cmd".equals(event.get("type").getAsString())) {
                    String commandText = event.has("text") ? event.get("text").getAsString() : "";
                    if (!commandText.isEmpty()) {
                        executeCommand(commandText);
                    }
                }
            } catch (Exception ignored) {
                // Ignore non-JSON or malformed events
            }
        }
    }

    public void executeCommand(String json) {
        if (json == null || json.isEmpty())
            return;

        try {
            com.google.gson.JsonObject cmd = org.firstinspires.ftc.teamcode.jules.bridge.util.GsonCompat.parse(json)
                    .getAsJsonObject();
            String type = cmd.has("type") ? cmd.get("type").getAsString() : "";

            // Handle "type": "motor"/"servo" (Low-Level)
            // String type = cmd.has("type") ? cmd.get("type").getAsString() : ""; //
            // Already defined above
            if ("motor".equals(type)) {
                String id = cmd.has("id") ? cmd.get("id").getAsString() : "";
                DcMotor motor = scanner.getMotor(id);
                if (motor != null) {
                    if (cmd.has("power")) {
                        motor.setPower(cmd.get("power").getAsDouble());
                    }
                }
                return;
            } else if ("servo".equals(type)) {
                String id = cmd.has("id") ? cmd.get("id").getAsString() : "";
                com.qualcomm.robotcore.hardware.Servo servo = scanner.getServo(id);
                if (servo != null) {
                    if (cmd.has("position")) {
                        servo.setPosition(cmd.get("position").getAsDouble());
                    }
                }
                return;
            }

            // Handle "name": "drive", "args": {...} (High-Level)
            if (cmd.has("name") && "drive".equals(cmd.get("name").getAsString())) {
                if (cmd.has("args")) {
                    com.google.gson.JsonObject args = cmd.get("args").getAsJsonObject();
                    // Mapping: t = throttle (y), p = pivot/turn (rx), s = strafe (x)
                    double t = args.has("t") ? args.get("t").getAsDouble() : 0.0;
                    double p = args.has("p") ? args.get("p").getAsDouble() : 0.0;
                    double s = args.has("s") ? args.get("s").getAsDouble() : 0.0; // Optional strafe

                    // Mecanum mixer
                    double y = -t; // Forward is usually negative Y in gamepad, but positive here?
                                   // Let's assume input t is positive forward.
                                   // Robot.setDrivePowers usually expects:
                                   // lf = y + x + rx
                                   // If t is 0.5 (forward), we want positive power?
                                   // In TeleOp: y = -gamepad1.left_stick_y. Up on stick is -1. So y is 1.
                                   // So t should be positive for forward.

                    double rx = p;
                    double x = s;

                    double denominator = Math.max(Math.abs(t) + Math.abs(x) + Math.abs(rx), 1.0);
                    double lf = (t + x + rx) / denominator;
                    double lr = (t - x + rx) / denominator;
                    double rf = (t - x - rx) / denominator;
                    double rr = (t + x - rx) / denominator;

                    setDrivePowers(lf, lr, rf, rr);

                    // NOTE: duration_ms handling requires async scheduling which isn't implemented
                    // yet.
                    // This command sets immediate power. The sender (LLM) should send a stop
                    // command (t=0) later,
                    // or we need a non-blocking timer in processCommands().
                }
            }
        } catch (Exception e) {
            RobotLog.e(TAG, "Command execution failed: " + e.getMessage());
        }
    }

    public void update() {
        if (follower != null) {
            follower.update();
        }
    }

    public com.google.gson.JsonObject getTelemetryData() {
        com.google.gson.JsonObject data = new com.google.gson.JsonObject();

        // Battery
        if (batterySensor != null) {
            data.addProperty("vbatt", batterySensor.getVoltage());
        }

        // IMU
        if (imu != null) {
            try {
                double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                data.addProperty("heading", heading);
            } catch (Exception ignored) {
            }
        }

        // Pedro
        if (JulesConstants.USE_PEDRO_PATHING) {
            data.addProperty("pedro_active", (follower != null));
            if (follower != null) {
                Pose pose = follower.getPose();
                if (pose != null) {
                    data.addProperty("x", pose.getX());
                    data.addProperty("y", pose.getY());
                    data.addProperty("h", Math.toDegrees(pose.getHeading()));
                }
            }
        } else {
            data.addProperty("pedro_active", false);
        }

        return data;
    }
}
