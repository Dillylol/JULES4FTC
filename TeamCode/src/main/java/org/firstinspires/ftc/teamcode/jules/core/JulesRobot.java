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
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.jules.bridge.JulesBridgeManager;
import org.firstinspires.ftc.teamcode.jules.bridge.JulesStreamBus;
import org.firstinspires.ftc.teamcode.jules.link.JulesLinkManager;
import org.firstinspires.ftc.teamcode.jules.link.JulesWsClient;
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
    public final JulesPathInterpreter pathInterpreter = new JulesPathInterpreter();

    // JULES Link
    public JulesLinkManager linkManager;

    // Dynamic Hardware Scanner
    public final JulesHardwareScanner scanner = new JulesHardwareScanner();

    // Bridge Manager Fields
    private JulesBridgeManager bridgeManager;
    private JulesStreamBus streamBus;
    private JulesStreamBus.Subscription subscription;

    // Auto-stop timer: when > 0, stop motors after this epoch ms
    public long driveStopTime = 0;

    private com.qualcomm.robotcore.eventloop.opmode.OpMode opMode;

    public JulesRobot(com.qualcomm.robotcore.eventloop.opmode.OpMode opMode) {
        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;
    }

    /**
     * Legacy constructor. Cannot use JulesLinkManager fully without OpMode.
     */
    public JulesRobot(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.opMode = null; // LinkManager will fail to init fully
    }

    public void init() {
        initBridge();
        initHardware(); // Initialize motors, sensors, IMU
        // initLinkManager(); // Disabled: Robot is now Server-only via
        // JulesBridgeManager
        // Dynamic scan
        scanner.scan(hardwareMap);

        // Advertise manifest if bridge is ready
        if (bridgeManager != null) {
            publishManifest();
        }
    }

    public void stop() {
        if (linkManager != null) {
            linkManager.stop();
        }
    }

    private void initHardware() {
        try {
            leftFront = hardwareMap.tryGet(DcMotor.class, JulesConstants.Motors.FRONT_LEFT);
            leftRear = hardwareMap.tryGet(DcMotor.class, JulesConstants.Motors.BACK_LEFT);
            rightFront = hardwareMap.tryGet(DcMotor.class, JulesConstants.Motors.FRONT_RIGHT);
            rightRear = hardwareMap.tryGet(DcMotor.class, JulesConstants.Motors.BACK_RIGHT);

            if (leftFront != null)
                leftFront.setDirection(JulesConstants.Motors.FRONT_LEFT_DIR);
            if (leftRear != null)
                leftRear.setDirection(JulesConstants.Motors.BACK_LEFT_DIR);
            if (rightFront != null)
                rightFront.setDirection(JulesConstants.Motors.FRONT_RIGHT_DIR);
            if (rightRear != null)
                rightRear.setDirection(JulesConstants.Motors.BACK_RIGHT_DIR);

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

    private void initLinkManager() {
        try {
            linkManager = new JulesLinkManager();
            // We need to pass the OpMode. But JulesRobot is instantiated inside OpMode.
            // We don't have reference to 'this' OpMode here comfortably unless passed in
            // constructor?
            // JulesRobot ctor takes hardwareMap and telemetry.
            // We might need to change JulesRobot ctor or assume we can pass null for now if
            // LinkManager handles it?
            // LinkManager.init(OpMode, ...)
            // Use 'null' for OpMode for now if not strictly required, OR pass it if
            // possible.
            // Wait, JulesMasterController extends OpMode.
            // Let's modify JulesRobot to take OpMode in constructor?
            // For now, simpler:
            if (opMode == null) {
                RobotLog.w(TAG, "Cannot init LinkManager: OpMode is null");
                return;
            }
            linkManager.init(opMode, hardwareMap, telemetry, opMode.gamepad1, opMode.gamepad2);
            linkManager.start();

            // Register command listener
            linkManager.onConnected(() -> {
                publishManifest();
            });

            linkManager.addCommandListener((type, payload) -> {
                // Bridge the new command listener to the existing executeCommand logic
                // Construct a synthetic event JSON or pass payload if structure matches
                // executeCommand expects "type", "payload" etc. inside the JSON object.
                // The 'payload' arg here IS the JSON object of the message?
                // No, onCommand(String type, JsonObject payload)
                // We can reconstruct the full object or adapt executeCommand.

                com.google.gson.JsonObject event = new com.google.gson.JsonObject();
                event.addProperty("type", type);
                // If payload is the full object or sub-part? WsClient says: send(obj).
                // handleInbound parses obj. type = obj.type.
                // onCommand(type, obj). So 'payload' IS the full message object.
                executeCommand(payload.toString());
            });
        } catch (Exception e) {
            RobotLog.e(TAG, "Link Init Failed", e);
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

                // Inject Battery Sensor
                try {
                    com.qualcomm.robotcore.hardware.VoltageSensor vs = hardwareMap.voltageSensor.iterator().next();
                    bridgeManager.setBatterySensor(vs);
                } catch (Exception ignored) {
                }

                // Ensure it starts
                bridgeManager.start(null, null);
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
        if (bridgeManager != null && streamBus != null) {
            streamBus.publishJsonLine(json);
        } else if (linkManager != null) {
            linkManager.sendNdjson(json);
        }
    }

    public void publishManifest() {
        if (bridgeManager != null) {
            // Ensure bridge has the latest scanner reference
            bridgeManager.setHardwareScanner(scanner);

            // Inject Pedro Pathing capability into manifest before publishing
            if (scanner != null) {
                com.google.gson.JsonObject manifest = scanner.getManifest();
                manifest.addProperty("type", "manifest");

                // Advertise Pedro Pathing capability
                manifest.addProperty("pedro_pathing", JulesConstants.USE_PEDRO_PATHING);
                manifest.addProperty("pedro_active", (follower != null));

                // Tag drivetrain motors so UI can warn about conflicts
                com.google.gson.JsonArray dtMotors = new com.google.gson.JsonArray();
                dtMotors.add(JulesConstants.Motors.FRONT_LEFT);
                dtMotors.add(JulesConstants.Motors.FRONT_RIGHT);
                dtMotors.add(JulesConstants.Motors.BACK_LEFT);
                dtMotors.add(JulesConstants.Motors.BACK_RIGHT);
                manifest.add("drivetrain_motors", dtMotors);

                if (JulesConstants.USE_PEDRO_PATHING) {
                    com.google.gson.JsonObject pathCaps = new com.google.gson.JsonObject();
                    pathCaps.addProperty("linear", true);
                    pathCaps.addProperty("tangential", true);
                    pathCaps.addProperty("constant", true);
                    manifest.add("path_capabilities", pathCaps);
                    manifest.addProperty("drive_mode", "pedro_pathing");
                } else {
                    manifest.addProperty("drive_mode", "power_time");
                }

                JulesStreamBus bus = bridgeManager.getStreamBus();
                if (bus != null) {
                    bus.publishJsonLine(manifest.toString());
                }
            } else {
                bridgeManager.publishManifest();
            }
        } else if (scanner != null) {
            // Fallback for legacy or debugging
            com.google.gson.JsonObject manifest = scanner.getManifest();
            manifest.addProperty("type", "manifest");
            manifest.addProperty("pedro_pathing", JulesConstants.USE_PEDRO_PATHING);
            manifest.addProperty("pedro_active", (follower != null));
            manifest.addProperty("ts", System.currentTimeMillis());
            publish(manifest.toString());
        }
    }

    /**
     * Poll and execute commands from the JULES Bridge (via JulesCommand mailbox).
     * Call this in your OpMode loop().
     */
    public void processCommands() {
        String cmd = org.firstinspires.ftc.teamcode.jules.bridge.JulesCommand.getAndClearCommand();
        if (cmd == null || cmd.isEmpty())
            return;

        com.qualcomm.robotcore.util.RobotLog.i("JulesRobot", "Processing command: " + cmd);

        try {
            // JSON payload → route through executeCommand()
            if (cmd.trim().startsWith("{")) {
                executeCommand(cmd);
                return;
            }

            // Legacy string commands
            if (cmd.startsWith("DRIVE_")) {
                String[] parts = cmd.split("_");
                if (parts.length >= 4) {
                    String direction = parts[1];
                    double time = Double.parseDouble(parts[2].replace("T", ""));
                    double power = Double.parseDouble(parts[3].replace("P", ""));
                    switch (direction.toUpperCase()) {
                        case "FORWARD":
                            setDrivePowers(power, power, power, power);
                            break;
                        case "BACKWARD":
                            setDrivePowers(-power, -power, -power, -power);
                            break;
                        case "LEFT":
                            setDrivePowers(-power, power, power, -power);
                            break;
                        case "RIGHT":
                            setDrivePowers(power, -power, -power, power);
                            break;
                    }
                }
            } else if ("STOP".equalsIgnoreCase(cmd)) {
                setDrivePowers(0, 0, 0, 0);
            }
        } catch (Exception e) {
            com.qualcomm.robotcore.util.RobotLog.e("JulesRobot", "Command error: " + e.getMessage());
        }
    }

    /**
     * Call in loop() to check auto-stop timer.
     */
    public void checkAutoStop() {
        if (driveStopTime > 0 && System.currentTimeMillis() >= driveStopTime) {
            setDrivePowers(0, 0, 0, 0);
            driveStopTime = 0;
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
            if ("motor".equals(type)) {
                String id = cmd.has("id") ? cmd.get("id").getAsString() : "";
                DcMotor motor = scanner.getMotor(id);
                if (motor != null) {
                    if (cmd.has("power")) {
                        motor.setPower(cmd.get("power").getAsDouble());
                    }
                } else {
                    String msg = "Motor '" + id + "' not found. Available: " + scanner.motors.keySet();
                    RobotLog.w(TAG, msg);
                    if (bridgeManager != null && streamBus != null) {
                        com.google.gson.JsonObject err = new com.google.gson.JsonObject();
                        err.addProperty("type", "log");
                        err.addProperty("level", "warn");
                        err.addProperty("message", msg);
                        streamBus.publishJsonLine(err.toString());
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
                } else {
                    String msg = "Servo '" + id + "' not found. Available: " + scanner.servos.keySet();
                    RobotLog.w(TAG, msg);
                    if (bridgeManager != null && streamBus != null) {
                        com.google.gson.JsonObject err = new com.google.gson.JsonObject();
                        err.addProperty("type", "log");
                        err.addProperty("level", "warn");
                        err.addProperty("message", msg);
                        streamBus.publishJsonLine(err.toString());
                    }
                }
                return;
            }

            // Handle "name": "<command>", "args": {...} (High-Level)
            String name = cmd.has("name") ? cmd.get("name").getAsString() : "";
            com.google.gson.JsonObject args = (cmd.has("args") && cmd.get("args").isJsonObject())
                    ? cmd.get("args").getAsJsonObject()
                    : new com.google.gson.JsonObject();

            // Read duration for auto-stop
            long durationMs = args.has("duration_ms") ? args.get("duration_ms").getAsLong() : 0;

            switch (name.toLowerCase()) {
                case "drive": {
                    double t = args.has("t") ? args.get("t").getAsDouble() : 0.0;
                    double p = args.has("p") ? args.get("p").getAsDouble() : 0.0;
                    double s = args.has("s") ? args.get("s").getAsDouble() : 0.0;

                    double y = t, rx = p, x = s;
                    double denom = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
                    double lf = (y + x + rx) / denom;
                    double lr = (y - x + rx) / denom;
                    double rf = (y - x - rx) / denom;
                    double rr = (y + x - rx) / denom;
                    setDrivePowers(lf, lr, rf, rr);
                    break;
                }
                case "strafe": {
                    double speed = args.has("speed") ? args.get("speed").getAsDouble() : 0.4;
                    // Strafe: positive speed = right, negative = left
                    // Mecanum strafe right: LF-, LR+, RF+, RR-
                    setDrivePowers(-speed, speed, speed, -speed);
                    break;
                }
                case "turn": {
                    double speed = args.has("speed") ? args.get("speed").getAsDouble() : 0.3;
                    // Turn: positive speed = right, negative = left
                    // In-place turn right: LF-, LR-, RF+, RR+
                    setDrivePowers(-speed, -speed, speed, speed);
                    break;
                }
                case "stop": {
                    setDrivePowers(0, 0, 0, 0);
                    driveStopTime = 0; // Cancel any pending auto-stop
                    return;
                }
                default:
                    break;
            }

            // ───── Path Commands (Pedro Pathing Dynamic) ─────
            switch (name.toLowerCase()) {
                case "path_start": {
                    pathInterpreter.reset();
                    RobotLog.i(TAG, "Path interpreter: reset");
                    return;
                }
                case "path_add": {
                    String pt = args.has("pathType") ? args.get("pathType").getAsString() : "linear";
                    double px = args.has("x") ? args.get("x").getAsDouble() : 0;
                    double py = args.has("y") ? args.get("y").getAsDouble() : 0;
                    double sh = args.has("startHeading") ? args.get("startHeading").getAsDouble() : 0;
                    double eh = args.has("endHeading") ? args.get("endHeading").getAsDouble() : 0;
                    double hd = args.has("heading") ? args.get("heading").getAsDouble() : 0;
                    boolean rev = args.has("reverse") && args.get("reverse").getAsBoolean();
                    pathInterpreter.addSegment(pt, px, py, sh, eh, hd, rev);
                    RobotLog.i(TAG, "Path interpreter: added " + pt + " segment");
                    return;
                }
                case "path_follow": {
                    if (follower != null) {
                        boolean ok = pathInterpreter.execute(follower);
                        RobotLog.i(TAG, "Path interpreter: follow -> " + (ok ? "success" : "failed"));
                    } else {
                        RobotLog.w(TAG, "Path interpreter: follower is null, cannot execute");
                    }
                    return;
                }
                default:
                    // Not a path command — check if it's an unrecognized drive command
                    if (!name.isEmpty()
                            && !"drive".equals(name) && !"strafe".equals(name)
                            && !"turn".equals(name) && !"stop".equals(name)) {
                        RobotLog.w(TAG, "Unknown command name: " + name);
                    }
                    break;
            }

            // Set auto-stop timer if duration provided
            if (durationMs > 0) {
                driveStopTime = System.currentTimeMillis() + durationMs;
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
            data.addProperty("path_segments", pathInterpreter.getSegmentCount());
            data.addProperty("path_building", pathInterpreter.isActive());
        } else {
            data.addProperty("pedro_active", false);
        }

        return data;
    }
}
