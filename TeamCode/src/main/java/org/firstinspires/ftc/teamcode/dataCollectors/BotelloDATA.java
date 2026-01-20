package org.firstinspires.ftc.teamcode.dataCollectors;

import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "BotelloDATA")
public class BotelloDATA extends OpMode {

    // Drive motors (stay DcMotor – no velocity reads)
    private DcMotor BackL, BackR, FrontL, FrontR;

    // Mechanisms (use DcMotorEx so we can read velocity)
    private DcMotorEx Intake, Wheel, Wheel2;

    // Auto-boot servo (stays lowered, kicks only when wheel velocity >= threshold)
    private Servo Boot;

    // Panels telemetry
    private final Telemetry panels = PanelsTelemetry.INSTANCE.getFtcTelemetry();

    // IMU for field-centric
    private IMU imu;

    // Toggles
    private boolean isWheelOn = false; // B toggles this
    private boolean bWasPressed = false;

    // Button edges for tuning
    private boolean rbPrev = false, lbPrev = false, duPrev = false, ddPrev = false, dlPrev = false, drPrev = false,
            startPrev = false;

    // REV-41-1600 (28 CPR) * gear ratio => ticks per output revolution
    private static final double MOTOR_ENCODER_CPR = 28.0;
    private static final double INTAKE_GEAR_RATIO = 20.0; // TODO: set yours
    private static final double WHEEL_GEAR_RATIO = 1.0; // TODO: set yours

    private static final double INTAKE_TPR = MOTOR_ENCODER_CPR * INTAKE_GEAR_RATIO; // 560 for 20:1
    private static final double WHEEL_TPR = MOTOR_ENCODER_CPR * WHEEL_GEAR_RATIO; // 28 for 1:1

    // ===== Servo config =====
    private static final double BOOT_STOW_POS = 0.0; // resting (default)
    private static final double BOOT_KICK_POS = 0.5; // extended only when wheel is fast enough

    // Velocity thresholds for boot hysteresis (RPM -> TPS)
    private static final double BOOT_ON_RPM = 2000.0; // extend at/above this wheel speed
    private static final double BOOT_OFF_RPM = 1500.0; // retract at/below this wheel speed
    private static final double BOOT_ON_TPS = (BOOT_ON_RPM / 60.0) * WHEEL_TPR;
    private static final double BOOT_OFF_TPS = (BOOT_OFF_RPM / 60.0) * WHEEL_TPR;

    private boolean bootIsExtended = false; // track last servo state for hysteresis

    // ===== Tunable RPM hold =====
    private double targetWheelRPM = 3550.0; // starting setpoint (under-load target)
    private static final double STEP_SMALL = 25; // RB/LB
    private static final double STEP_MED = 100; // DPAD L/R
    private static final double STEP_BIG = 250; // DPAD U/D
    private static final double RPM_MIN = 0.0;
    private static final double RPM_MAX = 6000.0; // sanity clamp

    @Override
    public void init() {
        // Map drive
        BackL = hardwareMap.get(DcMotor.class, "lr");
        BackR = hardwareMap.get(DcMotor.class, "rr");
        FrontL = hardwareMap.get(DcMotor.class, "lf");
        FrontR = hardwareMap.get(DcMotor.class, "rf");

        // Map mechanisms as DcMotorEx
        Intake = hardwareMap.get(DcMotorEx.class, "Intake");
        Wheel = hardwareMap.get(DcMotorEx.class, "Wheel");
        Wheel2 = hardwareMap.get(DcMotorEx.class, "Wheel2");

        // Map servo
        Boot = hardwareMap.get(Servo.class, "boot");

        // Directions
        FrontL.setDirection(DcMotor.Direction.REVERSE);
        BackL.setDirection(DcMotor.Direction.REVERSE);
        FrontR.setDirection(DcMotor.Direction.REVERSE);
        BackR.setDirection(DcMotor.Direction.REVERSE);

        Intake.setDirection(DcMotor.Direction.REVERSE);
        Wheel.setDirection(DcMotor.Direction.REVERSE);
        Wheel2.setDirection(DcMotor.Direction.REVERSE);

        // Brakes
        DcMotor.ZeroPowerBehavior brake = DcMotor.ZeroPowerBehavior.BRAKE;
        FrontL.setZeroPowerBehavior(brake);
        FrontR.setZeroPowerBehavior(brake);
        BackL.setZeroPowerBehavior(brake);
        BackR.setZeroPowerBehavior(brake);
        Intake.setZeroPowerBehavior(brake);
        // Wheel: RUN_USING_ENCODER + setVelocity handles holding; BRAKE is fine here
        // too
        Wheel.setZeroPowerBehavior(brake);
        Wheel2.setZeroPowerBehavior(brake);

        // Use encoders for velocity control
        Wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Wheel2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Wheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Wheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // (Optional) You can tune PIDF here if needed (SDK default is usually OK)
        // Wheel.setVelocityPIDFCoefficients(kP, kI, kD, kF);

        // Servo default
        Boot.setPosition(BOOT_STOW_POS);
        bootIsExtended = false;

        // IMU setup
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters params = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(params);

        panels.addData("Status", "Init complete: RPM Hold enabled");
        panels.update();
    }

    @Override
    public void loop() {
        // Reset yaw with D-Pad Up (hold)
        if (gamepad1.left_bumper) {
            imu.resetYaw(); // hidden combo (LS+DU) to avoid conflicts with tuning
        }
        duPrev = gamepad1.dpad_up;

        // Field-centric drive
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1; // compensate strafing
        double rx = gamepad1.right_stick_x;

        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
        double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

        double denom = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);
        double fl = (rotY + rotX + rx) / denom;
        double bl = (rotY - rotX + rx) / denom;
        double fr = (rotY - rotX - rx) / denom;
        double br = (rotY + rotX - rx) / denom;

        FrontL.setPower(fl);
        FrontR.setPower(fr);
        BackL.setPower(bl);
        BackR.setPower(br);

        // ===== Mechanisms =====
        // (1) Intake control: A = normal intake, X = reverse, else off
        double intakeCmd = gamepad1.a ? 1.0 : (gamepad1.x ? -1.0 : 0.0);
        Intake.setPower(intakeCmd);

        // (2) Wheel toggle on B:
        if (gamepad1.b && !bWasPressed)
            isWheelOn = !isWheelOn;
        bWasPressed = gamepad1.b;

        // ===== Tunable RPM setpoint (edge-detected) =====
        // RB/LB: ±25
        if (gamepad1.right_bumper && !rbPrev)
            adjustTargetRPM(+STEP_SMALL);
        if (gamepad1.left_bumper && !lbPrev)
            adjustTargetRPM(-STEP_SMALL);
        rbPrev = gamepad1.right_bumper;
        lbPrev = gamepad1.left_bumper;

        // DPad Right/Left: ±100
        if (gamepad1.dpad_right && !drPrev)
            adjustTargetRPM(+STEP_MED);
        if (gamepad1.dpad_left && !dlPrev)
            adjustTargetRPM(-STEP_MED);
        drPrev = gamepad1.dpad_right;
        dlPrev = gamepad1.dpad_left;

        // DPad Up/Down: ±250
        if (gamepad1.dpad_up && !duPrev)
            adjustTargetRPM(+STEP_BIG);
        if (gamepad1.dpad_down && !ddPrev)
            adjustTargetRPM(-STEP_BIG);
        ddPrev = gamepad1.dpad_down;

        // START: snap to preset (useful between cycles)
        if (gamepad1.start && !startPrev)
            targetWheelRPM = 3550.0;
        startPrev = gamepad1.start;

        // ===== Apply hold or stop =====
        final double wheelTps = safeAssemblyVel();
        final double wheelRpm = toRPM(wheelTps, WHEEL_TPR);

        if (isWheelOn) {
            // Ensure RUN_USING_ENCODER for velocity control
            ensureRunUsingEncoder(Wheel);
            ensureRunUsingEncoder(Wheel2);
            double targetTps = (targetWheelRPM / 60.0) * WHEEL_TPR;
            setWheelVelocity(targetTps); // built-in PIDF holds TPS
        } else {
            setWheelPower(0.0);
        }

        // ===== Auto-Boot Servo based on Wheel velocity =====
        if (!bootIsExtended && wheelTps >= BOOT_ON_TPS) {
            Boot.setPosition(BOOT_KICK_POS);
            bootIsExtended = true;
        } else if (bootIsExtended && wheelTps <= BOOT_OFF_TPS) {
            Boot.setPosition(BOOT_STOW_POS);
            bootIsExtended = false;
        }

        // Telemetry/graphs
        double intakeTps = safeVel(Intake);
        double intakeRpm = toRPM(intakeTps, INTAKE_TPR);
        double batteryV = getBatteryVoltage();

        panels.addData("Wheel_ON", isWheelOn);
        panels.addData("Wheel_RPM_target", targetWheelRPM);
        panels.addData("Wheel_RPM_meas", wheelRpm);
        panels.addData("Wheel_TPS_meas", wheelTps);
        panels.addData("Intake_RPM", intakeRpm);
        panels.addData("Battery_V", batteryV);
        panels.addData("Boot_Pos", Boot.getPosition());
        panels.addData("BootExtended", bootIsExtended);
        panels.addData("Heading_deg", Math.toDegrees(heading));
        panels.addData("FL_power", fl);
        panels.addData("FR_power", fr);
        panels.addData("BL_power", bl);
        panels.addData("BR_power", br);
        panels.update();
    }

    private void adjustTargetRPM(double delta) {
        targetWheelRPM = clamp(targetWheelRPM + delta, RPM_MIN, RPM_MAX);
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private void setWheelPower(double power) {
        Wheel.setPower(power);
        if (Wheel2 != null) {
            Wheel2.setPower(power);
        }
    }

    private void setWheelVelocity(double ticksPerSecond) {
        setVelocitySafe(Wheel, ticksPerSecond);
        setVelocitySafe(Wheel2, ticksPerSecond);
    }

    private void ensureRunUsingEncoder(DcMotorEx motor) {
        if (motor != null && motor.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    private double safeAssemblyVel() {
        double v1 = safeVel(Wheel);
        double v2 = safeVel(Wheel2);
        return (Wheel2 != null) ? 0.5 * (v1 + v2) : v1;
    }

    private static double safeVel(DcMotorEx m) {
        if (m == null) {
            return 0.0;
        }
        try {
            return m.getVelocity();
        } catch (Exception e) {
            return 0.0;
        }
    }

    private static void setVelocitySafe(DcMotorEx motor, double ticksPerSecond) {
        if (motor == null) {
            return;
        }
        try {
            motor.setVelocity(ticksPerSecond);
        } catch (Exception ignored) {
        }
    }

    private static double toRPM(double tps, double tpr) {
        return (tpr <= 0) ? 0.0 : (tps / tpr) * 60.0;
    }

    private double getBatteryVoltage() {
        double min = Double.POSITIVE_INFINITY;
        for (VoltageSensor s : hardwareMap.getAll(VoltageSensor.class)) {
            double v = s.getVoltage();
            if (v > 0)
                min = Math.min(min, v);
        }
        return (min == Double.POSITIVE_INFINITY) ? 0.0 : min;
    }
}
// Certified Dylen Vasquez Design
