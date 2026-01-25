package org.firstinspires.ftc.teamcode.steele27303;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.common.BjornConstants;
import org.firstinspires.ftc.teamcode.common.shooter.ShooterController;

/**
 * Base TeleOp - Subsystems Only (Reduced Latency Architecture)
 * 
 * Contains ONLY:
 * - Intake Control (A = In, X = Out)
 * - ShooterController (Flywheel RPM, Readiness)
 * - LED Status (Green = Ready, Red = Not Ready)
 * - ShooterController (Flywheel RPM, Readiness)
 * - LED Status (Green = Ready, Red = Not Ready)
 * - Auto Grip Spin when Ready
 * - Manual Grip Spin (RT)
 * 
 * Drive, IMU, Pedro, Camera are in subclasses for reduced latency.
 */
public abstract class BjornTeleBase extends OpMode {

    // --- Abstract ---
    protected abstract String getAllianceName();

    protected abstract int getGoalTagId();

    // --- Subsystem Hardware ---
    protected DcMotorEx intake, wheel, wheel2;
    protected CRServo grip1, grip2;
    protected DigitalChannel led1Green, led1Red, led2Green, led2Red;

    // --- Systems ---
    protected ShooterController shooterController;

    // --- State ---
    protected boolean shooterActive = false;
    protected boolean shooterIdle = false;

    // --- Input State ---
    private boolean bPrev = false;
    private boolean b2Prev = false;
    private boolean y2Prev = false;
    private boolean rtPrev = false;

    // --- Constants ---
    protected static final double SHOOTER_RPM = 2800.0;
    protected static final double SHOOTER_IDLE_RPM = 1000.0;

    /**
     * Subclasses MUST call this in their init() method.
     */
    protected void initSubsystems() {
        // 1. Subsystem Motors
        intake = hardwareMap.get(DcMotorEx.class, BjornConstants.Motors.INTAKE);
        wheel = hardwareMap.get(DcMotorEx.class, BjornConstants.Motors.WHEEL);
        wheel2 = hardwareMap.get(DcMotorEx.class, BjornConstants.Motors.WHEEL2);

        intake.setDirection(BjornConstants.Motors.INTAKE_DIRECTION);
        wheel.setDirection(BjornConstants.Motors.WHEEL_DIRECTION);
        wheel2.setDirection(BjornConstants.Motors.WHEEL2_DIRECTION);

        // 2. Grip Servos
        grip1 = hardwareMap.get(CRServo.class, BjornConstants.Motors.GRIP1);
        grip2 = hardwareMap.get(CRServo.class, BjornConstants.Motors.GRIP2);
        grip1.setDirection(BjornConstants.Motors.GRIP1_DIRECTION);
        grip2.setDirection(BjornConstants.Motors.GRIP2_DIRECTION);

        // 4. LEDs
        led1Green = hardwareMap.get(DigitalChannel.class, "led1_green");
        led1Red = hardwareMap.get(DigitalChannel.class, "led1_red");
        led2Green = hardwareMap.get(DigitalChannel.class, "led2_green");
        led2Red = hardwareMap.get(DigitalChannel.class, "led2_red");

        led1Green.setMode(DigitalChannel.Mode.OUTPUT);
        led1Red.setMode(DigitalChannel.Mode.OUTPUT);
        led2Green.setMode(DigitalChannel.Mode.OUTPUT);
        led2Red.setMode(DigitalChannel.Mode.OUTPUT);

        // 5. ShooterController
        VoltageSensor vSensor = null;
        for (VoltageSensor s : hardwareMap.getAll(VoltageSensor.class)) {
            if (s.getVoltage() > 0) {
                vSensor = s;
                break;
            }
        }
        shooterController = new ShooterController(wheel, wheel2, intake, null, vSensor);
    }

    /**
     * Subclasses MUST call this in their loop() method.
     */
    protected void updateSubsystems(long nowMs) {
        shooterController.update(nowMs);
        handleShooterToggle(nowMs);
        handleIntake();
        handleGrips(nowMs);

        boolean isReady = shooterController.isReady(nowMs);
        setLedStatus(isReady);
    }

    // --- Input Handlers ---

    private void handleShooterToggle(long nowMs) {
        boolean b = gamepad1.b;
        if (b && !bPrev) {
            shooterActive = !shooterActive;
            if (!shooterActive) shooterIdle = false; // Force Stop
        }
        bPrev = b;

        boolean b2 = gamepad2.b;
        if (b2 && !b2Prev) {
            shooterActive = !shooterActive;
            if (!shooterActive) shooterIdle = false; // Force Stop
        }
        b2Prev = b2;

        boolean y2 = gamepad2.y;
        if (y2 && !y2Prev)
            shooterIdle = !shooterIdle;
        y2Prev = y2;

        if (shooterActive) {
            shooterController.setTargetRpm(SHOOTER_RPM, nowMs);
        } else if (shooterIdle) {
            shooterController.setTargetRpm(SHOOTER_IDLE_RPM, nowMs);
        } else {
            shooterController.stop(nowMs);
        }
    }

    private void handleIntake() {
        double intakePower = 0.0;
        if (gamepad1.a)
            intakePower = 1.0;
        else if (gamepad1.x)
            intakePower = -1.0;
        intake.setPower(intakePower);
    }

    private void handleGrips(long nowMs) {
        boolean isReady = shooterController.isReady(nowMs);
        boolean rtPressed = (gamepad1.right_trigger > 0.5) || (gamepad2.right_trigger > 0.5);
        boolean ltPressed = (gamepad1.left_trigger > 0.5) || (gamepad2.left_trigger > 0.5);

        // Priority 1: Outtake (Manual LT or X) - Overrides Intake
        if (ltPressed || gamepad1.x) {
            grip1.setPower(-1.0);
            grip2.setPower(-1.0);
            return;
        }

        // Priority 2: Intake (Manual RT Only - "Shooting")
        if (rtPressed) {
            grip1.setPower(1.0);
            grip2.setPower(1.0);
            return;
        }

        // Priority 3: Idle
        grip1.setPower(0.0);
        grip2.setPower(0.0);
    }

    // --- LED Status ---

    protected void setLedStatus(boolean isReady) {
        if (isReady) {
            led1Green.setState(true);
            led1Red.setState(false);
            led2Green.setState(true);
            led2Red.setState(false);
        } else {
            led1Green.setState(false);
            led1Red.setState(true);
            led2Green.setState(false);
            led2Red.setState(true);
        }
    }

    // --- Telemetry Helper ---
    protected void addSubsystemTelemetry(long nowMs) {
        boolean isReady = shooterController.isReady(nowMs);
        telemetry.addData("Shooter", shooterActive ? "ON" : (shooterIdle ? "IDLE" : "OFF"));
        telemetry.addData("RPM", "%.0f / %.0f", shooterController.getMeasuredRpm(),
                (double) shooterController.getTargetRpm());
        telemetry.addData("Ready", isReady);
    }
}
