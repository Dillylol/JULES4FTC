package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Central place for robot-wide hardware configuration constants.
 * Only contains identifiers and simple defaults so both TeleOp and
 * Autonomous code can reference the same definitions.
 */
public final class BjornConstants {
    private BjornConstants() {
        // Utility class
    }

    public static final class Motors {
        private Motors() {
        }

        // Hardware names
        public static final String FRONT_LEFT = "lf";
        public static final String FRONT_RIGHT = "rf";
        public static final String BACK_LEFT = "lr";
        public static final String BACK_RIGHT = "rr";
        public static final String INTAKE = "Intake";
        public static final String WHEEL = "Wheel";
        public static final String WHEEL2 = "Wheel2";
        public static final String TURRET = "Turret";
        public static final String GRIP1 = "Grip1";
        public static final String GRIP2 = "Grip2";
        // public static final String BOOT = "boot"; // Removed

        // Default behaviors
        // public static final DcMotor.Direction DRIVE_DIRECTION =
        // DcMotor.Direction.FORWARD; // Removed
        public static final DcMotor.ZeroPowerBehavior DRIVE_ZERO_POWER = DcMotor.ZeroPowerBehavior.BRAKE;

        public static final DcMotor.Direction INTAKE_DIRECTION = DcMotor.Direction.REVERSE;
        public static final DcMotor.ZeroPowerBehavior INTAKE_ZERO_POWER = DcMotor.ZeroPowerBehavior.BRAKE;
        public static final DcMotor.Direction WHEEL2_DIRECTION = DcMotor.Direction.FORWARD; // flip if needed
        public static final DcMotor.Direction WHEEL_DIRECTION = DcMotor.Direction.REVERSE; // flip if needed
        public static final DcMotor.Direction TURRET_DIRECTION = DcMotor.Direction.FORWARD;
        public static final DcMotor.ZeroPowerBehavior TURRET_ZERO_POWER = DcMotor.ZeroPowerBehavior.BRAKE;
        public static final DcMotorSimple.Direction GRIP1_DIRECTION = DcMotorSimple.Direction.REVERSE;
        public static final DcMotorSimple.Direction GRIP2_DIRECTION = DcMotorSimple.Direction.FORWARD;
    }

    public static final class GearRatios {
        private GearRatios() {
        }

        public static final double DRIVE = 1.0;
        public static final double INTAKE = 1.0;
        public static final double WHEEL = 1.0;
        public static final double TURRET = 1.0;
    }

    public static final class Sensors {
        private Sensors() {
        }

        public static final String IMU = "imu";
        public static final String TOF_FRONT = "TOF";
    }

    public static final class Power {
        private Power() {
        }

        // Nominal battery voltage (12V system baseline)
        public static final double NOMINAL_BATT_V = 12.0;

        // RPM added per volt of sag for the shooter (trained by K-tuner)
        public static final double SHOOTER_K_V_RPM = 0.0; // keep 0.0 as default

        // Max RPM change per update step for ramping, to reduce current spikes
        public static final int SHOOTER_MAX_RPM_STEP_PER_UPDATE = 250; // tune as needed

        // S-Curve Ramp Tuning (Live Tunable)
        public static double RAMP_COEF = 0.00025;
        public static double RAMP_MIN_RATE = 400.0;

        // Dynamic RPM Calculation Constants
        public static final double SHOOTER_RPM_SLOPE_TOF = 116.4042383594456;
        public static final double SHOOTER_RPM_OFFSET_TOF = 2284.2966941424975;
        public static final double SHOOTER_RPM_SLOPE_CV = 116.4042383594456;
        public static final double SHOOTER_RPM_OFFSET_CV = 2284.2966941424975;

        public static final double SHOOTER_MIN_RPM = 2200.0; // Lower bound for valid shots
        public static final double SHOOTER_MAX_RPM = 4500.0; // Safety cap
    }

    public static final class Auto {
        private Auto() {
        }

        // End Poses for Auto-Drive (PedroPathing)
        // TODO: Verify these coordinates match your actual auto end positions
        public static final com.pedropathing.geometry.Pose BLUE_AUTO_END_POSE = new com.pedropathing.geometry.Pose(
                -30.8,
                46.8, Math.toRadians(-165));
        public static final com.pedropathing.geometry.Pose RED_AUTO_END_POSE = new com.pedropathing.geometry.Pose(27.2,
                61, Math.toRadians(-13.7));

        public static final com.pedropathing.geometry.Pose BLUE_AUTO_START_POSE = new com.pedropathing.geometry.Pose(0,
                0, Math.toRadians(265));
        public static final com.pedropathing.geometry.Pose RED_AUTO_START_POSE = new com.pedropathing.geometry.Pose(0,
                0, Math.toRadians(-85));
    }

    public static final class Servos {
        private Servos() {
        }

        public static final double BOOT_STOW = 0.0; // Retracted (0 degrees)
        public static final double BOOT_KICK = 0.5; // Extended (90 degrees)
    }

    public static final class AutoPoses {
        private AutoPoses() {
        }

        public static final class Blue {
            private Blue() {
            }

            public static final com.pedropathing.geometry.Pose START = new com.pedropathing.geometry.Pose(0, 0,
                    Math.toRadians(265));
            public static final com.pedropathing.geometry.Pose SHOOT_ZONE = new com.pedropathing.geometry.Pose(0, 25,
                    Math.toRadians(-85));
            public static final com.pedropathing.geometry.Pose GLANCE_POINT = new com.pedropathing.geometry.Pose(0, 30,
                    Math.toRadians(-85)); // TODO: Tune
            public static final com.pedropathing.geometry.Pose ROW_START = new com.pedropathing.geometry.Pose(10, 30,
                    Math.toRadians(-85)); // TODO: Tune
            public static final com.pedropathing.geometry.Pose ROW1 = new com.pedropathing.geometry.Pose(12, 36,
                    Math.toRadians(0)); // TODO: Tune
            public static final com.pedropathing.geometry.Pose ROW2 = new com.pedropathing.geometry.Pose(12, 24,
                    Math.toRadians(0)); // TODO: Tune
            public static final com.pedropathing.geometry.Pose ROW3 = new com.pedropathing.geometry.Pose(12, 12,
                    Math.toRadians(0)); // TODO: Tune
            public static final com.pedropathing.geometry.Pose ALIGN1 = new com.pedropathing.geometry.Pose(22.6, 37.7,
                    Math.toRadians(-58));
            public static final com.pedropathing.geometry.Pose GRAB1 = new com.pedropathing.geometry.Pose(32.9, 16,
                    Math.toRadians(-58));
            public static final com.pedropathing.geometry.Pose ALIGN1_BACK = new com.pedropathing.geometry.Pose(23, 31,
                    Math.toRadians(-58));
            public static final com.pedropathing.geometry.Pose PARK = new com.pedropathing.geometry.Pose(28.6, 48.7,
                    Math.toRadians(-145));
        }
    }
}
