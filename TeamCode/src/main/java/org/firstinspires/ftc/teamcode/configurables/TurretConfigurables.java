package org.firstinspires.ftc.teamcode.configurables;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;

/**
 * Turret PID Configuration - Tunable via Panels UI
 * 
 * These settings can be adjusted in real-time through the Panels dashboard.
 * Use for tuning the turret control loop during testing.
 */
@Configurable
public class TurretConfigurables {

    @Sorter(sort = 0)
    public static double kP = 0.03;

    @Sorter(sort = 1)
    public static double kI = 0.0;

    @Sorter(sort = 2)
    public static double kD = 0.0008;

    @Sorter(sort = 3)
    public static double feedForward = 0.0025;

    @Sorter(sort = 4)
    public static double deadband = 2.0;

    @Sorter(sort = 5)
    public static double maxPowerDelta = 0.04;
}
