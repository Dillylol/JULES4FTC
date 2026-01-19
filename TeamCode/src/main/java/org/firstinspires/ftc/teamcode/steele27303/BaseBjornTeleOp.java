package org.firstinspires.ftc.teamcode.steele27303;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.common.BjornHardware;
import org.firstinspires.ftc.teamcode.common.shooter.ShooterController;
import org.firstinspires.ftc.teamcode.jules.cv.AprilTagCamera;
import org.firstinspires.ftc.teamcode.jules.cv.AprilTagCamera.TagObservation;

import java.util.List;

/**
 * Condensed "Smart" Base TeleOp with Turret Gateway Pipeline.
 * - Subclasses define Alliance Tag ID (Blue/Red).
 * - Controls:
 *   - LS: Field Drive | RS: Turn
 *   - D-Pad Down: Retract Odometry / Reset IMU
 *   - D-Pad Up (Edge): Toggle Turret Mode (MANUAL <-> ACTIVE)
 *   - D-Pad L/R: Turret Manual (in Manual Mode)
 *   - A: Intake | X: Outtake
 *   - B: Shooter Ramp
 *   - RB: Set Waypoint | LT: Drive to Waypoint
 * - Auto-Fire:
 *   - If Shooter Ready AND Turret Locked AND Camera Sees Tag -> Fire Grips automatically.
 */
public abstract class BaseBjornTeleOp extends LinearOpMode {

    protected abstract int getTargetTagId();
    protected abstract String getAllianceName();

    protected BjornHardware hardware;
    protected ShooterController shooterController;
    protected AprilTagCamera camera;
    protected double turretPower = 0.0;

    protected static final double SHOOTER_RPM = 2800.0;
    protected static final double SHOOTER_IDLE_RPM = 1000.0;
    protected static final double DRIVE_SPEED = 1.0;
    protected static final double TURN_SPEED = 0.7;
    protected static final double TURRET_MANUAL_POWER = 0.4;
    
    // Logic State
    protected boolean turretActiveMode = false; // Default Manual
    protected boolean shooterActive = false;
    protected boolean shooterIdle = false;
    protected boolean bPrev = false;
    protected boolean b2Prev = false; // G2 Toggle
    protected boolean y2Prev = false; // G2 Idle Toggle
    protected boolean dpadUpPrev = false;
    protected boolean dpadUp2Prev = false; // G2 Toggle
    
    // Simplified Turret PID
    protected static final double TURRET_KP = 0.02; 
    protected static final double TURRET_LIMIT_DEG = 170.0;

    @Override
    public void runOpMode() throws InterruptedException {
        // Abstract base: no implementation here.
        // Subclasses must implement.
    }

    protected void initCommon() {
        // 1. Initialize Hardware
        hardware = BjornHardware.forTeleOp(hardwareMap);
        
        // 2. Initialize Subsystems
        shooterController = new ShooterController(
            hardware.wheel, 
            hardware.wheel2, 
            hardware.intake, 
            hardware, 
            hardware.getVoltageSensor()
        );
        
        // Camera
        camera = new AprilTagCamera();
        camera.start(hardwareMap, null);

        telemetry.addLine("BjornTeleOp: " + getAllianceName());
        telemetry.addLine("Target Tag ID: " + getTargetTagId());
        telemetry.addLine("Controls:");
        telemetry.addLine("  LS: Field Drive | RS: Turn");
        telemetry.addLine("  D-Pad Down: Reset IMU");
        telemetry.addLine("  D-Pad Up: Toggle Turret Mode (Manual/Active)");
        telemetry.addLine("  D-Pad L/R: Turret Manual");
        telemetry.addLine("  A: In | X: Out | B: Shooter");
        telemetry.update();
    }

    protected void calculateDrive(double x, double y, double rx) {
        // Brake Light Logic: "Reverse" is usually positive Y on gamepad (stick down), 
        // effectively meaning y input is negative in robot frame if y = -gamepad.y
        // Assuming standard: y = -gamepad1.left_stick_y. 
        // Stick Down -> gamepad positive -> y negative -> Reverse.
        // Threshold: y < -0.1
        setBrakeStatus(y < -0.1);

        double botHeading = hardware.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        
        // Counteract imperfect strafing
        rotX = rotX * 1.1; 

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        hardware.frontLeft.setPower((rotY + rotX + rx) / denominator);
        hardware.backLeft.setPower((rotY - rotX + rx) / denominator);
        hardware.frontRight.setPower((rotY - rotX - rx) / denominator);
        hardware.backRight.setPower((rotY + rotX - rx) / denominator);
    }
    
    protected void setBrakeStatus(boolean reversing) {
        // If Reversing: Red ON, Green OFF
        // Else: Green ON, Red OFF
        boolean redState = reversing;
        boolean greenState = !reversing;
        
        if (hardware.brake1Red != null) hardware.brake1Red.setState(redState);
        if (hardware.brake1Green != null) hardware.brake1Green.setState(greenState);
        
        if (hardware.brake2Red != null) hardware.brake2Red.setState(redState);
        if (hardware.brake2Green != null) hardware.brake2Green.setState(greenState);
    }
    
    protected void setLedStatus(boolean isReady) {
         // Green = Ready (ledGreen true, ledRed false)
         // Red = Not Ready (ledGreen false, ledRed true)
         if (isReady) {
             if (hardware.led1Green != null) hardware.led1Green.setState(true);
             if (hardware.led1Red != null) hardware.led1Red.setState(false);
             
             if (hardware.led2Green != null) hardware.led2Green.setState(true);
             if (hardware.led2Red != null) hardware.led2Red.setState(false);
         } else {
             if (hardware.led1Green != null) hardware.led1Green.setState(false);
             if (hardware.led1Red != null) hardware.led1Red.setState(true);
             
             if (hardware.led2Green != null) hardware.led2Green.setState(false);
             if (hardware.led2Red != null) hardware.led2Red.setState(true);
         }
    }
}
