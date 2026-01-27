package org.firstinspires.ftc.teamcode.steele27303;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.common.BjornConstants;
import org.firstinspires.ftc.teamcode.common.BjornHardware;
import org.firstinspires.ftc.teamcode.common.shooter.TeleOpShooter;

/**
 * Base TeleOp - Subsystems Only (Reduced Latency Architecture)
 * 
 * Contains ONLY:
 * - Intake Control (A = In, X = Out)
 * - TeleOpShooter (Flywheel RPM, Readiness, ToF)
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
    protected BjornHardware hardware;
    protected TeleOpShooter shooter;

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
        // 1. Initialize Hardware Wrapper
        hardware = BjornHardware.forTeleOp(hardwareMap);

        // 2. Alias fields for subclass compatibility
        intake = hardware.intake;
        wheel = hardware.wheel;
        wheel2 = hardware.wheel2;
        grip1 = hardware.grip1;
        grip2 = hardware.grip2;

        led1Green = hardware.led1Green;
        led1Red = hardware.led1Red;
        led2Green = hardware.led2Green;
        led2Red = hardware.led2Red;

        // 3. Initialize Shooter
        shooter = new TeleOpShooter(hardware);
    }

    /**
     * Subclasses MUST call this in their loop() method.
     */
    protected void updateSubsystems(long nowMs) {
        shooter.update();
        handleShooterToggle();
        handleShooterCvInput();
        handleIntake();
        handleGrips();

        // LED status is handled inside TeleOpShooter now, but if we need manual
        // override:
        // shooter.update() calls updateLeds() internally.
    }

    // --- Input Handlers ---

    private void handleShooterToggle() {
        boolean b = gamepad1.b;
        if (b && !bPrev) {
            shooter.toggle();
        }
        bPrev = b;

        boolean b2 = gamepad2.b;
        if (b2 && !b2Prev) {
            shooter.toggle();
        }
        b2Prev = b2;

        boolean y2 = gamepad2.y;
        if (y2 && !y2Prev) {
            shooter.toggleIdle();
        }
        y2Prev = y2;
    }

    private boolean g2DpadLeftPrev = false;

    private void handleShooterCvInput() {
        boolean dpadLeft = gamepad2.dpad_left;
        if (dpadLeft && !g2DpadLeftPrev) {
            shooter.toggleCv();
        }
        g2DpadLeftPrev = dpadLeft;
    }

    private void handleIntake() {
        double intakePower = 0.0;
        if (gamepad1.a)
            intakePower = 1.0;
        else if (gamepad1.x)
            intakePower = -1.0;

        if (intake != null)
            intake.setPower(intakePower);
    }

    private void handleGrips() {
        boolean rtPressed = (gamepad1.right_trigger > 0.5) || (gamepad2.right_trigger > 0.5);
        boolean ltPressed = (gamepad1.left_trigger > 0.5) || (gamepad2.left_trigger > 0.5);

        // Priority 1: Outtake (Manual LT or X)
        if (ltPressed || gamepad1.x) {
            if (grip1 != null)
                grip1.setPower(-1.0);
            if (grip2 != null)
                grip2.setPower(-1.0);
            return;
        }

        // Priority 2: Intake (Manual RT Only - "Shooting")
        if (rtPressed) {
            if (grip1 != null)
                grip1.setPower(1.0);
            if (grip2 != null)
                grip2.setPower(1.0);
            return;
        }

        // Priority 3: Idle (managed by Shooter? No, Shooter manages LEDs. Grips are
        // manual in TeleOp)
        if (grip1 != null)
            grip1.setPower(0.0);
        if (grip2 != null)
            grip2.setPower(0.0);
    }

    // --- LED Status ---
    // Delegated to TeleOpShooter

    // --- Telemetry Helper ---
    protected void addSubsystemTelemetry(long nowMs) {
        telemetry.addData("Shooter", shooter.isActive() ? "ACTIVE" : (shooter.isIdle() ? "IDLE" : "OFF"));
        telemetry.addData("RPM", "%.0f / %d", shooter.getMeasuredRpm(), shooter.getTargetRpm());
        telemetry.addData("Dist", "%.1f in", shooter.getDistanceInches());
        telemetry.addData("Ready", shooter.isReady());
        telemetry.addData("CV", shooter.isCvEnabled() ? "ON" : "OFF");
    }
}
