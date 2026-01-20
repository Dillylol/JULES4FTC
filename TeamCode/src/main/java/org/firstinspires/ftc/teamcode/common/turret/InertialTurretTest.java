package org.firstinspires.ftc.teamcode.common.turret;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/**
 * Inertial Turret Test - IMU-Locked Heading (STANDALONE RIG VERSION)
 * 
 * Gear Ratio: 80:1 total
 * - Motor internal: 5:1 * 4:1 = 20:1
 * - External: 20T -> 20T -> 80T = 4:1
 * 
 * Controls:
 * - A: Lock turret to current field heading
 * - B: Reset target to 0° (forward)
 * - D-Pad Down: Reset IMU yaw
 * - Y: Reset turret angle
 * - Triggers: Manual turret control (when unlocked)
 * 
 * Limits: 20° to 160°
 */
@TeleOp(name = "TEST: Inertial Turret (Rig)", group = "Tests")
public class InertialTurretTest extends LinearOpMode {

    private DcMotorEx turret;
    private IMU imu;

    // Gear ratio: 80:1 total
    private static final double MOTOR_TICKS_PER_REV = 28.0;
    private static final double TURRET_GEAR_REDUCTION = 5.0 * 4.0 * 4.0; // 80:1
    private static final double TURRET_TICKS_PER_DEGREE = (MOTOR_TICKS_PER_REV * TURRET_GEAR_REDUCTION) / 360.0;

    // --- Tuning Constants ---
    public static double TURRET_KP = 0.03;
    public static double TURRET_KD = 0.0008;
    public static double ROBOT_ROTATION_FF_GAIN = 0.0025;

    // Deadband: Ignore small errors to reduce jitter
    public static double DEADBAND_DEG = 2.0;

    // Slew Rate: Max power change per loop to smooth output
    public static double MAX_POWER_DELTA = 0.04;

    // Limits
    public static double LIMIT_MIN = 20.0;
    public static double LIMIT_MAX = 160.0;

    // State
    private boolean lockActive = false;
    private double targetFieldHeading = 0.0;
    private double turretAngleDeg = 0.0;
    private int lastEncoderPos = 0;
    private double lastPower = 0.0;

    private boolean aPrev = false;
    private boolean bPrev = false;

    @Override
    public void runOpMode() {
        turret = hardwareMap.get(DcMotorEx.class, "Turret");
        turret.setDirection(DcMotor.Direction.FORWARD);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        telemetry.addLine("Inertial Turret Test (Rig)");
        telemetry.addLine("A: Lock | B: Reset to 0° | Y: Reset turret");
        telemetry.update();

        waitForStart();

        double lastTime = getRuntime();
        double lastError = 0;
        double lastRobotYaw = 0;
        lastEncoderPos = turret.getCurrentPosition();

        while (opModeIsActive()) {
            double now = getRuntime();
            double dt = now - lastTime;
            lastTime = now;

            // Update turret angle from encoder
            int currentPos = turret.getCurrentPosition();
            int deltaTicks = currentPos - lastEncoderPos;
            lastEncoderPos = currentPos;
            turretAngleDeg += deltaTicks / TURRET_TICKS_PER_DEGREE;

            // Get robot/rig rotation
            YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
            double robotYaw = angles.getYaw(AngleUnit.DEGREES);
            double robotRate = (dt > 0) ? (robotYaw - lastRobotYaw) / dt : 0.0;
            lastRobotYaw = robotYaw;

            // Input handling
            if (gamepad1.a && !aPrev) {
                lockActive = true;
                targetFieldHeading = robotYaw + turretAngleDeg;
            }
            aPrev = gamepad1.a;

            if (gamepad1.b && !bPrev) {
                lockActive = true;
                targetFieldHeading = 0.0;
            }
            bPrev = gamepad1.b;

            if (gamepad1.dpad_down) {
                imu.resetYaw();
            }

            if (gamepad1.y) {
                turretAngleDeg = 0.0;
                turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                lastEncoderPos = 0;
            }

            // Control Logic
            double turretPower = 0.0;

            if (lockActive) {
                double desiredRel = targetFieldHeading - robotYaw;
                double error = desiredRel - turretAngleDeg;

                // Apply deadband to reduce jitter
                if (Math.abs(error) < DEADBAND_DEG) {
                    error = 0;
                }

                // PD Controller
                double derivative = (dt > 0) ? (error - lastError) / dt : 0.0;
                double pid = (error * TURRET_KP) + (derivative * TURRET_KD);
                double ff = -robotRate * ROBOT_ROTATION_FF_GAIN;

                turretPower = pid + ff;
                lastError = error;
            } else {
                turretPower = (gamepad1.left_trigger - gamepad1.right_trigger) * 0.4;
            }

            // Apply Limits
            if (turretAngleDeg < LIMIT_MIN && turretPower < 0)
                turretPower = 0;
            if (turretAngleDeg > LIMIT_MAX && turretPower > 0)
                turretPower = 0;

            // Clamp and slew rate limit
            if (Double.isNaN(turretPower))
                turretPower = 0;
            turretPower = Range.clip(turretPower, -0.75, 0.75);

            double powerDelta = Range.clip(turretPower - lastPower, -MAX_POWER_DELTA, MAX_POWER_DELTA);
            turretPower = lastPower + powerDelta;
            lastPower = turretPower;

            turret.setPower(turretPower);

            // Telemetry
            telemetry.addData("Mode", lockActive ? "LOCKED" : "MANUAL");
            telemetry.addData("Target", "%.1f°", targetFieldHeading);
            telemetry.addData("Rig Yaw", "%.1f°", robotYaw);
            telemetry.addData("Turret", "%.1f°", turretAngleDeg);
            telemetry.addData("Power", "%.2f", turretPower);
            telemetry.addData("Deadband", "%.1f°", DEADBAND_DEG);
            telemetry.update();
        }
    }
}
