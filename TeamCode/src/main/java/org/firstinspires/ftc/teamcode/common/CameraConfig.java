package org.firstinspires.ftc.teamcode.common;

/**
 * Centralized camera + AprilTag configuration for the 2024-2025 DECODE field.
 * Tunable values can be edited and will persist during the app session.
 */
public final class CameraConfig {

    // Physical camera reference
    public static final String WEBCAM_NAME = "Webcam 1";

    // Tag IDs for the season
    public static final int BLUE_GOAL_TAG_ID = 20;
    public static final int OBELISK_TAG_ID_21 = 21;
    public static final int OBELISK_TAG_ID_22 = 22;
    public static final int OBELISK_TAG_ID_23 = 23;
    public static final int RED_GOAL_TAG_ID = 24;

    // Camera intrinsics for Logitech C920 at 1280x720
    // Camera intrinsics for Logitech C920 at 1280x720
    // Tunable via CameraConfigurables/Dashboard
    public static double FX = 930.0;
    public static double FY = 930.0;
    public static double CX = 640.0;
    public static double CY = 360.0;

    // --- Tunable Camera Settings (Updated by CV Tuner) ---
    // Recommended for 30fps: Exposure=6, Gain=250, Decimation=3
    public static int TUNED_EXPOSURE = 6; // Lower = less blur, darker
    public static int TUNED_GAIN = 250; // Higher = brighter, more noise
    public static int TUNED_DECIMATION = 3; // 3=30fps, 2=22fps, 1=10fps

    // FTC tag size (6.5 inches)
    public static final double TAG_SIZE_METERS = 0.165;

    public static final String CLASS_BLUE_GOAL = "blue_goal";
    public static final String CLASS_RED_GOAL = "red_goal";
    public static final String CLASS_OBELISK = "obelisk";
    public static final String CLASS_UNKNOWN = "unknown";

    // --- Control Mapping ---
    // 1.0 = Add yaw to current. -1.0 = Subtract yaw from current.
    // Use this to flip direction if turret moves AWAY from tag.
    public static final double CAMERA_TO_TURRET_SCALAR = 1.0;

    private CameraConfig() {
    }

    public static String classify(int id) {
        if (id == BLUE_GOAL_TAG_ID) {
            return CLASS_BLUE_GOAL;
        }
        if (id == RED_GOAL_TAG_ID) {
            return CLASS_RED_GOAL;
        }
        if (id == OBELISK_TAG_ID_21 || id == OBELISK_TAG_ID_22 || id == OBELISK_TAG_ID_23) {
            return CLASS_OBELISK;
        }
        return CLASS_UNKNOWN;
    }

    public static boolean isGoalTag(int id) {
        return id == BLUE_GOAL_TAG_ID || id == RED_GOAL_TAG_ID;
    }
}
