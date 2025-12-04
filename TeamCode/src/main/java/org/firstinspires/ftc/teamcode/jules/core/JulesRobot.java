package org.firstinspires.ftc.teamcode.jules.core;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.jules.bridge.JulesBridgeManager;
import org.firstinspires.ftc.teamcode.jules.bridge.JulesStreamBus;
import org.firstinspires.ftc.teamcode.jules.constants.JulesConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * JULES Robot Core Abstraction.
 * <p>
 * This class handles:
 * 1. Hardware Initialization
 * 2. PedroPathing Setup (if enabled)
 * 3. JULES Bridge Communication
 * 4. Basic Drive Control
 */
public class JulesRobot {
    private static final String TAG = "JulesRobot";

    public final HardwareMap hardwareMap;
    public final Telemetry telemetry;

    // Hardware
    public DcMotor leftFront, leftRear, rightFront, rightRear;
    public VoltageSensor batterySensor;

    // Pedro
    public Follower follower;

    // JULES Bridge
    public JulesBridgeManager bridgeManager;
    public JulesStreamBus streamBus;

    public JulesRobot(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }

    public void init() {
        initHardware();
        initPedro();
        initBridge();
    }

    private void initHardware() {
        try {
            leftFront = hardwareMap.get(DcMotor.class, JulesConstants.Motors.FRONT_LEFT);
            leftRear = hardwareMap.get(DcMotor.class, JulesConstants.Motors.BACK_LEFT);
            rightFront = hardwareMap.get(DcMotor.class, JulesConstants.Motors.FRONT_RIGHT);
            rightRear = hardwareMap.get(DcMotor.class, JulesConstants.Motors.BACK_RIGHT);

            leftFront.setDirection(JulesConstants.Motors.LEFT_DIR);
            leftRear.setDirection(JulesConstants.Motors.LEFT_DIR);
            rightFront.setDirection(JulesConstants.Motors.RIGHT_DIR);
            rightRear.setDirection(JulesConstants.Motors.RIGHT_DIR);

            // Battery
            for (VoltageSensor sensor : hardwareMap.getAll(VoltageSensor.class)) {
                batterySensor = sensor;
                break; // Use the first one
            }
        } catch (Exception e) {
            telemetry.addData("Init Error", "Hardware missing: " + e.getMessage());
            RobotLog.ee(TAG, "Hardware Init Failed", e);
        }
    }

    private void initPedro() {
        if (JulesConstants.USE_ODOMETRY) {
            try {
                follower = Constants.createFollower(hardwareMap);
                follower.setStartingPose(new Pose(0, 0, 0)); // Default start
            } catch (Exception e) {
                RobotLog.ee(TAG, "Pedro Init Failed", e);
                telemetry.addData("Pedro Error", e.getMessage());
            }
        }
    }

    private void initBridge() {
        try {
            bridgeManager = JulesBridgeManager.getInstance();
            if (bridgeManager != null) {
                bridgeManager.prepare(hardwareMap.appContext);
                streamBus = bridgeManager.getStreamBus();
            }
        } catch (Exception e) {
            RobotLog.ee(TAG, "Bridge Init Failed", e);
        }
    }

    public void start() {
        if (follower != null) {
            follower.startTeleopDrive();
        }
    }

    public void update() {
        if (follower != null) {
            follower.update();
        }
        // Add other periodic updates here
    }

    public void setDrivePowers(double lf, double lr, double rf, double rr) {
        if (JulesConstants.USE_BATTERY_OPTIMIZATION && batterySensor != null) {
            double voltage = batterySensor.getVoltage();
            double scale = Math.min(1.0, voltage / 12.0); // Simple scaling
            lf *= scale;
            lr *= scale;
            rf *= scale;
            rr *= scale;
        }
        
        if (follower != null) {
            // If using Pedro, we might want to use its drive method, but for raw power:
            // Pedro doesn't expose raw motor set easily without breaking its loop if we are in auto.
            // But for TeleOp, we can use setTeleOpDrive.
            // For raw motor control (tuners), we might need direct access.
            // Let's use direct access for now if follower isn't busy.
             leftFront.setPower(lf);
             leftRear.setPower(lr);
             rightFront.setPower(rf);
             rightRear.setPower(rr);
        } else {
            leftFront.setPower(lf);
            leftRear.setPower(lr);
            rightFront.setPower(rf);
            rightRear.setPower(rr);
        }
    }
    
    public void publish(String json) {
        if (streamBus != null) {
            streamBus.publishJsonLine(json);
        }
    }
}
