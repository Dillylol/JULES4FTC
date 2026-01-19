package org.firstinspires.ftc.teamcode.common.turret;

import com.qualcomm.robotcore.util.Range;

/**
 * Translates Turret Estimations into Motor Power using a PID Controller.
 * Also determines if the turret is "Locked On" to the target.
 */
public class TurretControl {

    private final TurretEstimator estimator;
    
    // PID Constants (Tunable)
    private static final double kP = 0.035;
    private static final double kI = 0.0; // Keep simple for now
    private static final double kD = 0.002;
    private static final double FEEDFORWARD = 0.0; // Static friction overcome?
    
    // Tolerances
    private static final double LOCK_TOLERANCE_DEG = 3.0; // +/- 3 degrees
    
    // State
    private double targetAngle = 0.0; // Default to 0 (Center/Aligned)
    private double lastError = 0.0;
    private long lastTimeMs = 0;
    
    public TurretControl(TurretEstimator estimator) {
        this.estimator = estimator;
    }
    
    /**
     * Updates the control loop and returns the motor power.
     * @param nowMs Current timestamp.
     * @return Motor power (-1.0 to 1.0).
     */
    public double update(long nowMs) {
        if (lastTimeMs == 0) {
            lastTimeMs = nowMs;
            return 0.0;
        }
        
        double dt = (nowMs - lastTimeMs) / 1000.0;
        lastTimeMs = nowMs;
        
        // Error = Target - Actual (Belief)
        // If Target is 0 and Belief is +10 (Right), Error is -10.
        // We need to move Left.
        double belief = estimator.getBelief();
        double error = targetAngle - belief;
        
        // P Term
        double pOut = kP * error;
        
        // D Term
        double derivative = (dt > 0) ? (error - lastError) / dt : 0.0;
        double dOut = kD * derivative;
        lastError = error;
        
        // Total Output
        double output = pOut + dOut + (Math.signum(error) * FEEDFORWARD);
        
        // Invert if necessary? 
        // If Error is -10, pOut is Negative. Motor needs to move Left (Negative power usually).
        // Matches standard setup.
        
        return Range.clip(output, -1.0, 1.0);
    }
    
    public void setTargetAngle(double angle) {
        this.targetAngle = angle;
    }
    
    public boolean isLocked() {
        // Locked if error is small
        double error = Math.abs(targetAngle - estimator.getBelief());
        // Could also allow lock if Uncertainty is HIGH? No, logic is "Camera Locked On".
        // "Camera Locked On" implies we have a belief powered by camera updates.
        // But TurretControl only knows Belief.
        // OpMode validates if Camera is actually seeing tag.
        
        return error <= LOCK_TOLERANCE_DEG;
    }
    
    public double getError() {
        return targetAngle - estimator.getBelief();
    }
}
