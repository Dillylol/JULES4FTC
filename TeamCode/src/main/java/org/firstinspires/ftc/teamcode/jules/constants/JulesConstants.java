package org.firstinspires.ftc.teamcode.jules.constants;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Standardized Constants for JULES Framework.
 * <p>
 * These constants define the hardware map names and configuration flags for the
 * robot.
 * Users should modify this file to match their specific robot configuration.
 */
public class JulesConstants {

    // ---------------- Configuration Flags ----------------
    public static boolean USE_ODOMETRY = true; // Set to false if no odometry pods
    public static boolean USE_WEBCAM = true; // Set to false if no camera
    public static boolean USE_BATTERY_OPTIMIZATION = true; // Scale power based on voltage
    public static boolean USE_PEDRO_PATHING = false; // Toggle for Pedro Pathing

    // ---------------- Hardware Map Names ----------------
    public static final class Motors {
        public static final String FRONT_LEFT = "lf";
        public static final String FRONT_RIGHT = "rf";
        public static final String BACK_LEFT = "lr";
        public static final String BACK_RIGHT = "rr";

        // Directions
        public static final DcMotorSimple.Direction LEFT_DIR = DcMotorSimple.Direction.REVERSE;
        public static final DcMotorSimple.Direction RIGHT_DIR = DcMotorSimple.Direction.FORWARD;
    }

    public static final class Sensors {
        public static final String IMU = "imu";
        public static final String WEBCAM = "Webcam 1";
    }

    public static final class IMU {
        public static final RevHubOrientationOnRobot.LogoFacingDirection LOGO_DIR = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        public static final RevHubOrientationOnRobot.UsbFacingDirection USB_DIR = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
    }

    // ---------------- Default Tuning Values ----------------
    // These are placeholders. The Tuner OpModes will help find the real values.
    public static final class Tuning {
        public static double DRIVE_P = 0.0;
        public static double DRIVE_I = 0.0;
        public static double DRIVE_D = 0.0;
        public static double DRIVE_F = 0.0;
    }
}
