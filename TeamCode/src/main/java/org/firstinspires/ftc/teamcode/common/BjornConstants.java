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
        public static final String FRONT_LEFT  = "lf";
        public static final String FRONT_RIGHT = "rf";
        public static final String BACK_LEFT   = "lr";
        public static final String BACK_RIGHT  = "rr";
        public static final String INTAKE      = "Intake";
        public static final String WHEEL       = "Wheel";
        public static final String WHEEL2       = "Wheel2";

        // Default behaviors
        public static final DcMotor.Direction DRIVE_DIRECTION = DcMotor.Direction.REVERSE;
        public static final DcMotor.ZeroPowerBehavior DRIVE_ZERO_POWER = DcMotor.ZeroPowerBehavior.BRAKE;

        public static final DcMotor.Direction INTAKE_DIRECTION = DcMotor.Direction.REVERSE;
        public static final DcMotor.ZeroPowerBehavior INTAKE_ZERO_POWER = DcMotor.ZeroPowerBehavior.BRAKE;
        public static final DcMotor.Direction WHEEL2_DIRECTION = DcMotor.Direction.REVERSE; // flip if needed
        public static final DcMotor.Direction WHEEL_DIRECTION = DcMotor.Direction.REVERSE; // flip if needed
    }

    public static final class Servos {
        private Servos() {
        }

        public static final String LIFT = "Lift";

        public static final double LIFT_LOWERED = 0.10;
        public static final double LIFT_RAISED  = 0.65;
    }

    public static final class Sensors {
        private Sensors() {
        }

        public static final String IMU       = "imu";
        public static final String TOF_FRONT = "TOF";
        public static final String vSensor = "vSensor";
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
    }
}
