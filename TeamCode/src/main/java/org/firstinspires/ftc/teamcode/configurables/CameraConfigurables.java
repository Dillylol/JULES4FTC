package org.firstinspires.ftc.teamcode.configurables;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;

/**
 * Camera Configuration - Tunable via Panels UI
 * 
 * These settings can be adjusted in real-time through the Panels dashboard.
 * Changes are applied immediately during OpMode execution.
 */
@Configurable
public class CameraConfigurables {

    @Sorter(sort = 0)
    public static int exposure = 6;

    @Sorter(sort = 1)
    public static int gain = 250;

    @Sorter(sort = 2)
    public static int decimation = 3;

    // Lens Intrinsics Tuning (Simulated in Tuner until restart)
    @Sorter(sort = 3)
    public static double fx = 930.0;

    @Sorter(sort = 4)
    public static double fy = 930.0;
}
