package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.List;
import org.firstinspires.ftc.teamcode.common.turret.TurretEstimator;

/**
 * Helper that owns references to the robot hardware and applies the shared
 * defaults.
 */
public final class BjornHardware {
    public final TurretEstimator turretEstimator;
    public final DcMotorEx frontLeft;
    public final DcMotorEx frontRight;
    public final DcMotorEx backLeft;
    public final DcMotorEx backRight;
    public final DcMotorEx intake;
    public final DcMotorEx wheel;
    public final DcMotorEx wheel2;
    public final DcMotorEx turret;
    public final CRServo grip1;
    public final CRServo grip2;
    // public final CRServo boot; // Removed
    public final DigitalChannel led1Green;
    public final DigitalChannel led1Red;
    public final DigitalChannel led2Green;
    public final DigitalChannel led2Red;
    public final DigitalChannel brake1Green, brake1Red;
    public final DigitalChannel brake2Green, brake2Red;
    public final DistanceSensor frontTof;
    public final IMU imu;
    // Boot
    public final Servo boot; // Renamed from lift
    public final VoltageSensor batterySensor; // Can be null in some configs, but usually filtered

    private double lastTurretPos = 0.0;

    // REV UltraPlanetary HD Hex Motor Specs
    // Base counts per revolution at the motor
    private static final double MOTOR_TICKS_PER_REV = 28.0;

    // User Configuration:
    // Cartridge 1: 5:1 (Actual: 5.23:1)
    // Cartridge 2: 4:1 (Actual: 3.61:1)
    // External Assembly: 4:1 (80T/20T)
    // Total = 5.23 * 3.61 * 4 = 75.52:1
    private static final double TURRET_GEAR_REDUCTION = 75.52;

    public static final double TURRET_TICKS_PER_DEGREE = (MOTOR_TICKS_PER_REV * TURRET_GEAR_REDUCTION) / 360.0;

    private BjornHardware(HardwareMap map) {
        frontLeft = wrap(map.get(DcMotorEx.class, BjornConstants.Motors.FRONT_LEFT), BjornConstants.GearRatios.DRIVE);
        frontRight = wrap(map.get(DcMotorEx.class, BjornConstants.Motors.FRONT_RIGHT), BjornConstants.GearRatios.DRIVE);
        backLeft = wrap(map.get(DcMotorEx.class, BjornConstants.Motors.BACK_LEFT), BjornConstants.GearRatios.DRIVE);
        backRight = wrap(map.get(DcMotorEx.class, BjornConstants.Motors.BACK_RIGHT), BjornConstants.GearRatios.DRIVE);

        intake = wrap(map.get(DcMotorEx.class, BjornConstants.Motors.INTAKE), BjornConstants.GearRatios.INTAKE);
        wheel = wrap(map.get(DcMotorEx.class, BjornConstants.Motors.WHEEL), BjornConstants.GearRatios.WHEEL);
        wheel2 = wrap(map.get(DcMotorEx.class, BjornConstants.Motors.WHEEL2), BjornConstants.GearRatios.WHEEL);
        turret = wrap(map.get(DcMotorEx.class, BjornConstants.Motors.TURRET), BjornConstants.GearRatios.TURRET);

        grip1 = map.get(CRServo.class, BjornConstants.Motors.GRIP1);
        grip2 = map.get(CRServo.class, BjornConstants.Motors.GRIP2);

        // Boot Removed
        // boot = map.get(CRServo.class, BjornConstants.Motors.BOOT);
        // boot.setDirection(CRServo.Direction.FORWARD);

        led1Green = map.get(DigitalChannel.class, "led1_green");
        led1Red = map.get(DigitalChannel.class, "led1_red");
        led2Green = map.get(DigitalChannel.class, "led2_green");
        led2Red = map.get(DigitalChannel.class, "led2_red");

        led1Green.setMode(DigitalChannel.Mode.OUTPUT);
        led1Red.setMode(DigitalChannel.Mode.OUTPUT);
        led2Green.setMode(DigitalChannel.Mode.OUTPUT);
        led2Red.setMode(DigitalChannel.Mode.OUTPUT);

        // Brake Lights
        brake1Green = map.get(DigitalChannel.class, "brake1_green");
        brake1Red = map.get(DigitalChannel.class, "brake1_red");
        brake2Green = map.get(DigitalChannel.class, "brake2_green");
        brake2Red = map.get(DigitalChannel.class, "brake2_red");

        brake1Green.setMode(DigitalChannel.Mode.OUTPUT);
        brake1Red.setMode(DigitalChannel.Mode.OUTPUT);
        brake2Green.setMode(DigitalChannel.Mode.OUTPUT);
        brake2Red.setMode(DigitalChannel.Mode.OUTPUT);

        // Default to Green (Not Braking)
        brake1Green.setState(true);
        brake1Red.setState(false);
        brake2Green.setState(true);
        brake2Red.setState(false);

        frontTof = map.get(DistanceSensor.class, BjornConstants.Sensors.TOF_FRONT);
        imu = map.get(IMU.class, BjornConstants.Sensors.IMU);

        // Boot
        boot = map.get(Servo.class, "boot");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        batterySensor = firstVoltageSensor(map);

        // Init Estimator: Assume 0 degrees at start, with 0 uncertainty (Hardware
        // Reset)
        turretEstimator = new org.firstinspires.ftc.teamcode.common.turret.TurretEstimator(0.0, 0.0);
    }

    public void updateEstimator() {
        double currentPos = turret.getCurrentPosition();
        double deltaTicks = currentPos - lastTurretPos;
        lastTurretPos = currentPos;

        double deltaDeg = deltaTicks / TURRET_TICKS_PER_DEGREE;
        turretEstimator.predict(deltaDeg);
    }

    /**
     * Call this when Camera sees a tag to correct belief
     */
    public void correctTurretBelief(double seenAngleDeg) {
        turretEstimator.correct(seenAngleDeg);
    }

    private static DcMotorEx wrap(DcMotorEx motor, double multiplier) {
        return new GearRatioMotor(motor, multiplier);
    }

    /**
     * Configure and return hardware for TeleOp use.
     */
    public static BjornHardware forTeleOp(HardwareMap map) {
        BjornHardware hardware = new BjornHardware(map);
        hardware.configureDriveMotors();
        hardware.configureMechanisms();
        hardware.resetWheelEncoder();
        return hardware;
    }

    /**
     * Configure and return hardware for Autonomous use.
     * Drive motors are left untouched for Pedro follower control.
     */
    public static BjornHardware forAutonomous(HardwareMap map) {
        BjornHardware hardware = new BjornHardware(map);
        hardware.configureMechanisms();
        return hardware;
    }

    private void configureDriveMotors() {
        // Front Motors REVERSED
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setZeroPowerBehavior(BjornConstants.Motors.DRIVE_ZERO_POWER);
        frontRight.setZeroPowerBehavior(BjornConstants.Motors.DRIVE_ZERO_POWER);

        // Back Motors FORWARD
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setZeroPowerBehavior(BjornConstants.Motors.DRIVE_ZERO_POWER);
        backRight.setZeroPowerBehavior(BjornConstants.Motors.DRIVE_ZERO_POWER);
    }
    // configureDriveMotor helper removed as it's no longer generic

    private void configureMechanisms() {
        intake.setDirection(BjornConstants.Motors.INTAKE_DIRECTION);
        intake.setZeroPowerBehavior(BjornConstants.Motors.INTAKE_ZERO_POWER);
        wheel2.setDirection(BjornConstants.Motors.WHEEL2_DIRECTION);
        wheel.setDirection(BjornConstants.Motors.WHEEL_DIRECTION);
        turret.setDirection(BjornConstants.Motors.TURRET_DIRECTION);
        turret.setZeroPowerBehavior(BjornConstants.Motors.TURRET_ZERO_POWER);
        grip1.setDirection(BjornConstants.Motors.GRIP1_DIRECTION);
        grip2.setDirection(BjornConstants.Motors.GRIP2_DIRECTION);
    }

    public void resetWheelEncoder() {
        wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheel2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Do we want to reset Turret encoder too? Probably.
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public double getBatteryVoltage() {
        if (batterySensor == null) {
            return BjornConstants.Power.NOMINAL_BATT_V;
        }
        try {
            return batterySensor.getVoltage();
        } catch (Exception ignored) {
            return BjornConstants.Power.NOMINAL_BATT_V;
        }
    }

    public VoltageSensor getVoltageSensor() {
        return batterySensor;
    }

    /**
     * Scales drive power based on battery voltage to prevent brownouts.
     * Logic derived from JULESData5.csv showing drops to ~10.9V.
     */
    public double getOptimizedDrivePower(double power) {
        double voltage = getBatteryVoltage();
        // Simple linear scaling: 100% at 12V+, linear drop to 50% at 10V
        double minV = 10.0;
        double maxV = 12.0;
        double scale = (voltage - minV) / (maxV - minV);
        scale = Math.max(0.5, Math.min(1.0, scale)); // Clamp between 0.5 and 1.0
        return power * scale;
    }

    private static VoltageSensor firstVoltageSensor(HardwareMap map) {
        List<VoltageSensor> sensors = map.getAll(VoltageSensor.class);
        return sensors.isEmpty() ? null : sensors.get(0);
    }
}
