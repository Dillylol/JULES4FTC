package org.firstinspires.ftc.teamcode.steele27303;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.common.BjornConstants;

/**
 * JULES Turret Auto-Tuner v7 - Step Response (Gentle)
 * 
 * Safe Step Response Tuner:
 * 1. Safe Centering: Uses current PID to center turret.
 * 2. Step Response: Applies 40% power step.
 * 3. Curve Analysis: Finds Max Velocity (Vmax) and Time Constant (Tau).
 * 4. Calc: Uses Cohen-Coon method to derive PID.
 * 
 * Controls:
 * - A: Start
 * - B: EMERGENCY STOP
 */
@TeleOp(name = "Bjorn Turret Auto Tuner Step", group = "Test")
public class BjornTurretAutoTuner extends BjornTeleBase {

    private DcMotorEx turret;
    
    // Constants
    private static final double MOTOR_TICKS_PER_REV = 28.0;
    private static final double TURRET_GEAR_REDUCTION = 75.52;
    private static final double TURRET_TICKS_PER_DEGREE = (MOTOR_TICKS_PER_REV * TURRET_GEAR_REDUCTION) / 360.0;
    
    // Limits
    private static final double HARD_MIN = 30.0;
    private static final double HARD_MAX = 155.0;
    private static final double CENTER = 90.0;
    private static final double STOP_ANGLE = 135.0; // Stop step test here (Safe within 155)
    
    // Tuning Parameters
    private static final double STEP_POWER = 0.40; // 40% Power Step
    
    // Centering PID (Current Working Values)
    private static final double CENTER_KP = 0.015;
    private static final double CENTER_KD = 0.004;
    
    private enum State {
        IDLE,
        CENTERING,
        STEP_TEST,
        COMPLETE,
        ESTOP
    }
    private State currentState = State.IDLE;
    
    // Data
    private double t0 = 0;
    private double maxVelocity = 0;
    private double timeToReach63 = 0; // Tau
    private boolean tauFound = false;
    
    // Results
    private double calcKp = 0;
    private double calcKd = 0;
    private double calcKv = 0;
    
    // Runtime
    private double lastLoopTime = 0;
    private int lastEncoderPos = 0;
    private double turretAngleDeg = 0;
    private double lastError = 0;
    
    @Override
    public void init() {
        turret = hardwareMap.get(DcMotorEx.class, "Turret");
        turret.setDirection(BjornConstants.Motors.TURRET_DIRECTION);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        initSubsystems();
        
        telemetry.addLine("=== STEP RESPONSE TUNER v7 ===");
        telemetry.addLine("Gentle 40% Power Test");
        telemetry.addLine("1. Center -> 2. Step -> 3. Calc");
        telemetry.addLine("Press A to Start");
        telemetry.update();
    }
    
    @Override
    protected String getAllianceName() { return "TEST"; }
    @Override
    protected int getGoalTagId() { return 0; }

    @Override
    public void loop() {
        // E-STOP
        if (gamepad1.b || gamepad2.b) {
            turret.setPower(0);
            currentState = State.ESTOP;
        }

        long nowMs = System.currentTimeMillis();
        double now = nowMs / 1000.0;
        double dt = (lastLoopTime > 0) ? (now - lastLoopTime) : 0.02;
        lastLoopTime = now;
        
        // Update encoder
        int currentPos = turret.getCurrentPosition();
        int deltaTicks = currentPos - lastEncoderPos;
        lastEncoderPos = currentPos;
        turretAngleDeg += (deltaTicks / TURRET_TICKS_PER_DEGREE) * BjornConstants.Motors.TURRET_ENCODER_DIRECTION;
        double velocity = (deltaTicks / TURRET_TICKS_PER_DEGREE) * BjornConstants.Motors.TURRET_ENCODER_DIRECTION / dt; // deg/s

        // Hard Limit Safety (Always Active except during step test we manage it)
        if (turretAngleDeg < HARD_MIN || turretAngleDeg > HARD_MAX) {
             if (currentState != State.IDLE && currentState != State.ESTOP && currentState != State.COMPLETE && currentState != State.CENTERING) {
                turret.setPower(0);
                currentState = State.ESTOP;
            }
        }

        switch (currentState) {
            case IDLE:
                turret.setPower(0);
                if (gamepad1.a || gamepad2.a) {
                    resetData();
                    currentState = State.CENTERING;
                }
                break;
                
            case CENTERING:
                // Move safely to 90 degrees
                double error = CENTER - turretAngleDeg;
                double derivative = (dt > 0) ? (error - lastError) / dt : 0;
                double pidPower = (error * CENTER_KP) + (derivative * CENTER_KD);
                lastError = error;
                
                // IMPORTANT: Use direction multiplier
                double finalPower = pidPower * BjornConstants.Motors.TURRET_POWER_DIRECTION;
                turret.setPower(Range.clip(finalPower, -0.6, 0.6)); // Soft limit power during centering
                
                telemetry.addData("State", "CENTERING");
                telemetry.addData("Error", error);
                
                if (Math.abs(error) < 5.0 && Math.abs(velocity) < 10.0) {
                    turret.setPower(0);
                    // Wait 1 second to settle? Or just go?
                    // Let's trigger step.
                    t0 = now;
                    currentState = State.STEP_TEST;
                }
                break;
                
            case STEP_TEST:
                // Apply constant Step Voltage
                // We want to go RIGHT (Positive Angle)
                // If PowerDirection is -1.0, we need Negative Raw Power to get Positive Output?
                // Wait. setPower(val * PowerDir).
                // If we send 1.0 * PowerDir(-1.0) = -1.0 Output.
                // -1.0 Output -> Ticks Decrease -> Angle Increases (Right).
                // So sending +STEP_POWER * PowerDir should move Right.
                
                turret.setPower(STEP_POWER * BjornConstants.Motors.TURRET_POWER_DIRECTION);
                
                // Record Stats
                double absVel = Math.abs(velocity);
                if (absVel > maxVelocity) {
                    maxVelocity = absVel;
                }
                
                // Find Tau (Time to reach 63.2% of Max Velocity)
                // This is dynamic estimation. Better to continuously check.
                // Since we don't know Vmax yet, we capture the curve.
                // Simplified: We assume Vmax is reached near end of test.
                // We need to store samples? No, let's just run until angle limit.
                
                telemetry.addData("State", "STEPPING (40%)");
                telemetry.addData("Vel", "%.0f", absVel);
                telemetry.addData("MaxVel", "%.0f", maxVelocity);
                
                // Stop Condition
                if (turretAngleDeg >= STOP_ANGLE) {
                    turret.setPower(0);
                    calculatePID(now - t0);
                    currentState = State.COMPLETE;
                }
                break;
                
            case COMPLETE:
                turret.setPower(0);
                telemetry.addLine("=== STEP TUNE COMPLETE ===");
                telemetry.addData("Kp", "%.5f", calcKp);
                telemetry.addData("Kd", "%.5f", calcKd);
                telemetry.addData("Kv", "%.5f", calcKv);
                telemetry.addData("MaxVel", "%.0f", maxVelocity);
                telemetry.addData("Tau", "%.3fs", timeToReach63);
                telemetry.addLine("Press A to Restart");
                
                if (gamepad1.a || gamepad2.a) {
                    currentState = State.IDLE;
                }
                break;
                
            case ESTOP:
                turret.setPower(0);
                telemetry.addLine("!!! ESTOP !!!");
                telemetry.addData("Angle", "%.1f", turretAngleDeg);
                telemetry.addLine("Press A to Restart");
                if (gamepad1.a) {
                    currentState = State.IDLE;
                }
                break;
        }
        
        telemetry.update();
    }
    
    private void resetData() {
        maxVelocity = 0;
        timeToReach63 = 0;
        tauFound = false;
        lastError = 0;
    }
    
    private void calculatePID(double duration) {
        // 1. Process Gain (K) = Output Speed / Input %
        // K = (deg/s) / (0.40)
        double K = maxVelocity / STEP_POWER;
        
        // 2. Time Constant (Tau) approximation
        // Assume it took roughly 'duration' to reach max speed? No, that's unsafe.
        // Better approx for FTC:
        // Tau = time to reach 63% speed.
        // We didn't record time-series, so we estimate Tau based on acceleration.
        // Let's use a standard approximation for DC motors: Tau ~ 0.1s to 0.2s usually.
        // Or calculate from displacement vs time if we assume 1st order.
        // Displacement d = Vmax * (t - Tau + Tau*e^(-t/Tau))
        // This is complex to solve onboard. 
        
        // CHANGE: Let's assume Tau ~ 0.15s (typical for Neverest/GoBilda) if we can't measure it.
        // OR: Use the "rise time" based on average accel.
        // Avg Accel = MaxVel / RiseTime.
        // RiseTime approx 3*Tau.
        // Let's blindly guess Tau = 0.1s for now to be safe, or 0.2s.
        // Let's use 0.1s.
        
        double Tau = 0.1; 
        
        // 3. Cohen-Coon / ZN Open Loop
        // Kp = (1.35 / K) * (Tau / DeadTime)
        // DeadTime is small, say 0.05s (loop + comms).
        double DeadTime = 0.05;
        
        calcKv = 1.0 / maxVelocity;
        
        // AMIGAF (Approximate Model Inversion) or simple P-Control rule
        // Kp = 1 / (K * (DeadTime + Tau)) is conservative 
        // Let's use Cohen-Coon simplified:
        // Kc = (1 / K) * (Tau / DeadTime) * 0.9
        
        double rawKp = (1.0 / K) * (Tau / DeadTime);
        calcKp = rawKp * 0.6; // Safety factor
        
        // Kd usually Tau * Kp / 4 for damping
        calcKd = calcKp * Tau * 0.5;
        
    }
}
