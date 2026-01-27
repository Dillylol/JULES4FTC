package org.firstinspires.ftc.teamcode.common.shooter;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.BjornConstants;
import org.firstinspires.ftc.teamcode.common.BjornHardware;

import org.firstinspires.ftc.teamcode.jules.cv.AprilTagCamera;
import org.firstinspires.ftc.teamcode.jules.cv.AprilTagCamera.TagObservation;
import java.util.List;

public class TeleOpShooter {
    private final BjornHardware hardware;
    private AprilTagCamera camera;
    private int goalTagId = -1;

    private boolean active = false;
    private boolean idle = false;
    private boolean useCv = false;

    private int targetRpm = 0;
    private double currentDistanceInches = 0.0;
    private double filteredRpm = 0.0;
    private static final double EMA_ALPHA = 0.2;
    private static final double TICKS_PER_REV = 28.0;
    private static final double READY_TOL_RPM = 100.0;
    private static final int IDLE_RPM = 2000;

    public TeleOpShooter(BjornHardware hardware) {
        this.hardware = hardware;
    }

    public void setCamera(AprilTagCamera camera, int goalTagId) {
        this.camera = camera;
        this.goalTagId = goalTagId;
    }

    public void toggle() {
        active = !active;
        if (active)
            idle = false; // If turning active, ensure not just idling (though logic below handles
                          // priority)
        if (!active) {
            // If turning off active, should we go to idle? Or full stop?
            // Usually Toggle = OFF.
        }
    }

    public void toggleIdle() {
        idle = !idle;
    }

    public void toggleCv() {
        useCv = !useCv;
    }

    public boolean isActive() {
        return active;
    }

    public boolean isIdle() {
        return idle;
    }

    public boolean isCvEnabled() {
        return useCv;
    }

    public int getTargetRpm() {
        return targetRpm;
    }

    public double getMeasuredRpm() {
        return filteredRpm;
    }

    public double getDistanceInches() {
        return currentDistanceInches;
    }

    public boolean isReady() {
        return active && targetRpm > 0 && Math.abs(filteredRpm - targetRpm) < READY_TOL_RPM;
    }

    public void update() {
        // 1. Update Measured RPM
        double measured = readRpm();
        filteredRpm = (EMA_ALPHA * measured) + ((1.0 - EMA_ALPHA) * filteredRpm);

        if (active) {
            double calculatedRpm = 0.0;
            boolean usingCvCalc = false;

            // 1. Try CV if enabled and available
            if (useCv && camera != null && goalTagId != -1) {
                List<TagObservation> detections = camera.pollDetections();
                for (TagObservation obs : detections) {
                    if (obs.id == goalTagId) {
                        double x = obs.x / 0.0254; // meters to inches
                        double y = obs.y / 0.0254;
                        double z = obs.z / 0.0254;
                        double distIn = Math.sqrt(x * x + y * y + z * z);
                        currentDistanceInches = distIn; // update metric
                        double rangeFt = distIn / 12.0;

                        double slope = BjornConstants.Power.SHOOTER_RPM_SLOPE_CV;
                        double offset = BjornConstants.Power.SHOOTER_RPM_OFFSET_CV;
                        calculatedRpm = (slope * rangeFt) + offset;
                        usingCvCalc = true;
                        break;
                    }
                }
            }

            // 2. Fallback to ToF if CV didn't result in calc
            if (!usingCvCalc) {
                if (hardware.frontTof != null) {
                    double distIn = hardware.frontTof.getDistance(DistanceUnit.INCH);
                    if (distIn > 100 || Double.isNaN(distIn)) {
                        // Bad data
                        currentDistanceInches = 0.0;
                        calculatedRpm = BjornConstants.Power.SHOOTER_MIN_RPM; // Default safely?
                    } else {
                        currentDistanceInches = distIn;
                        double rangeFt = distIn / 12.0;
                        double slope = BjornConstants.Power.SHOOTER_RPM_SLOPE_TOF;
                        double offset = BjornConstants.Power.SHOOTER_RPM_OFFSET_TOF;
                        calculatedRpm = (slope * rangeFt) + offset;
                    }
                }
            }

            // Clamp
            targetRpm = (int) Math.max(BjornConstants.Power.SHOOTER_MIN_RPM,
                    Math.min(calculatedRpm, BjornConstants.Power.SHOOTER_MAX_RPM));
            setFlywheelRpm(targetRpm);
        } else if (idle) {
            targetRpm = IDLE_RPM;
            setFlywheelRpm(targetRpm);
        } else {
            targetRpm = 0;
            setFlywheelRpm(0);
        }

        // 5. Update LEDs
        updateLeds();
    }

    private void setFlywheelRpm(int rpm) {
        double ticksPerSec = (rpm * TICKS_PER_REV) / 60.0;
        if (hardware.wheel != null)
            hardware.wheel.setVelocity(ticksPerSec);
        if (hardware.wheel2 != null)
            hardware.wheel2.setVelocity(ticksPerSec);
    }

    private double readRpm() {
        if (hardware.wheel == null)
            return 0.0;
        double v1 = hardware.wheel.getVelocity();
        double v2 = (hardware.wheel2 != null) ? hardware.wheel2.getVelocity() : v1;
        double avgV = (v1 + v2) / 2.0;
        return (avgV / TICKS_PER_REV) * 60.0;
    }

    private void updateLeds() {
        boolean ready = isReady();
        if (ready) {
            if (hardware.led1Green != null)
                hardware.led1Green.setState(true);
            if (hardware.led1Red != null)
                hardware.led1Red.setState(false);
            if (hardware.led2Green != null)
                hardware.led2Green.setState(true);
            if (hardware.led2Red != null)
                hardware.led2Red.setState(false);
        } else {
            // Not ready (Red)
            // If IDLE, maybe Yellow or Just Red? Usually Red implies "Don't Shoot".
            // Since Idle is just keeping it warm, Red is appropriate.
            if (hardware.led1Green != null)
                hardware.led1Green.setState(false);
            if (hardware.led1Red != null)
                hardware.led1Red.setState(true);
            if (hardware.led2Green != null)
                hardware.led2Green.setState(false);
            if (hardware.led2Red != null)
                hardware.led2Red.setState(true);
        }
    }
}
