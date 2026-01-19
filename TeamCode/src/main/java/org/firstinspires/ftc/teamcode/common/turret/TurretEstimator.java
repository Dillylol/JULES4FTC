package org.firstinspires.ftc.teamcode.common.turret;

/**
 * JULES Belief State Engine for the Turret (1D Scalar Kalman Filter).
 * 
 * Implements Recursive Bayesian Estimation (Section 2.1 of JULES Architecture)
 * to fuse Turret Encoder data (Prediction) with Camera observations (Update).
 * 
 * Math:
 * System State: x (Angle in degrees)
 * Uncertainty: P (Variance)
 * 
 * Prediction (Encoder):
 * x = x + u  (where u is encoder delta)
 * P = P + Q  (where Q is process noise/slip)
 * 
 * Update (Camera):
 * K = P / (P + R)  (Kalman Gain)
 * x = x + K * (z - x)
 * P = (1 - K) * P
 * 
 * The "Argument" (Conflict) is defined as the Innovation: delta = |z - x|
 */
public class TurretEstimator {

    // --- State Variables ---
    private double x; // Belief Mean (Degrees)
    private double P; // Belief Variance (Uncertainty)

    // --- Tuning Parameters ---
    // Q: Process Noise. How much uncertainty we add per prediction step.
    // Represents encoder slip, gear backlash, or drift.
    private static final double Q = 0.01; 

    // R: Measurement Noise. How much we distrust the Camera.
    // Higher R = Trust Encoder more. Lower R = Trust Camera more.
    private static final double R = 5.0; 

    // Conflict Threshold used for "Argument" logic
    private static final double CONFLICT_THRESHOLD_DEG = 10.0;

    // --- Metrics ---
    private double lastConflict = 0.0;
    private boolean isArguing = false; // True if conflict is high

    /**
     * Initialize the estimator.
     * @param initialGuessDegrees Starting angle (e.g., 0 for forward).
     * @param initialUncertainty Initial variance (e.g., 0.0 if sure, 100.0 if lost).
     */
    public TurretEstimator(double initialGuessDegrees, double initialUncertainty) {
        this.x = initialGuessDegrees;
        this.P = initialUncertainty;
    }

    /**
     * Prediction Step (Run this every loop).
     * projects the belief forward based on encoder movement.
     * @param deltaDegrees Change in encoder angle since last step.
     */
    public void predict(double deltaDegrees) {
        // x_hat = x + u
        x += deltaDegrees;
        
        // P = P + Q
        P += Q;
    }

    /**
     * Update/Correction Step (Run this when Camera sees a tag).
     * Fuses the observation into the belief.
     * @param observedAngleDegrees The absolute angle seen by the camera.
     */
    public void correct(double observedAngleDegrees) {
        // Calculate Innovation (The "Argument")
        double innovation = observedAngleDegrees - x;
        lastConflict = Math.abs(innovation);

        // Check for anomalies (JULES "Argument" Logic)
        if (lastConflict > CONFLICT_THRESHOLD_DEG) {
            isArguing = true;
            // Option A: Reject outlier (don't update)
            // Option B: Accept it anyway but log warning
            // For now, we will gate it: If P is low (we are confident), we reject crazy jumps.
            // If P is high (we are lost), we accept it.
            if (P < 10.0) {
                // We are confident, but sensor disagrees. Reject?
                // Or maybe drift is real?
                // Let's settle on standard KF for now, but flag it.
            }
        } else {
            isArguing = false;
        }

        // Kalman Gain
        // K = P / (P + R)
        double K = P / (P + R);

        // Update State
        // x = x + K * (z - x)
        x = x + K * innovation;

        // Update Uncertainty
        // P = (1 - K) * P
        P = (1.0 - K) * P;
    }

    /** 
     * @return The current best estimate of Turret Angle (Degrees). 
     */
    public double getBelief() {
        return x;
    }

    /**
     * @return The current uncertainty (Variance). Higher = Less Confident.
     */
    public double getUncertainty() {
        return P;
    }

    /**
     * @return The magnitude of the last conflict (Argument).
     */
    public double getConflict() {
        return lastConflict;
    }

    public boolean isArguing() {
        return isArguing;
    }
    
    /**
     * Force reset the belief state (e.g., at hard stop).
     */
    public void reset(double angle, double uncertainty) {
        this.x = angle;
        this.P = uncertainty;
    }
}
