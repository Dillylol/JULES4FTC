package org.firstinspires.ftc.teamcode.jules.shot;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.BjornConstants;

/**
 * Owns the flywheel control loop and single-shot sequencing.
 */
public final class ShooterController {

    public static final class ShotMetrics {
        public final double rpmAtFire;
        public final long timeToReadyMs;
        public final long fireTimestampMs;

        ShotMetrics(double rpmAtFire, long timeToReadyMs, long fireTimestampMs) {
            this.rpmAtFire = rpmAtFire;
            this.timeToReadyMs = timeToReadyMs;
            this.fireTimestampMs = fireTimestampMs;
        }
    }

    private static final double TICKS_PER_REV = 28.0;
    private static final double READY_TOL_RPM = 70.0;
    private static final long READY_SETTLE_MS = 300L;
    private static final double DIP_THRESHOLD_RPM = 150.0;
    private static final long DIP_WINDOW_MS = 220L;
    private static final long LOCKOUT_MS = 700L;
    private static final long LIFT_HOLD_MS = 350L;
    private static final long INTAKE_PULSE_NS = 200_000_000L;
    private static final double RPM_MIN = 1200.0;
    private static final double RPM_MAX = 3000.0;
    private static final double EMA_ALPHA = 0.2;

    private final DcMotorEx flywheel;
    private final DcMotorEx intake;
    private final Servo lift;

    private int targetRpm;
    private long targetSetMs;
    private long readyStartMs;
    private boolean readyLatched;
    private long readyAtMs;
    private long readyLatencyMs;
    private long readyLatencyAtFire;

    private double filteredRpm;

    private long lockoutUntilMs;
    private long fireCommandMs;
    private double rpmSnapshotAtFire;
    private boolean shotDetected;
    private ShotMetrics pendingShot;

    private long liftCloseAtMs;
    private long intakePulseEndNs;

    public ShooterController(@Nullable DcMotorEx flywheel,
                             @Nullable DcMotorEx intake,
                             @Nullable Servo lift) {
        this.flywheel = flywheel;
        this.intake = intake;
        this.lift = lift;
        this.targetRpm = 0;
        this.filteredRpm = 0.0;
        this.readyLatencyMs = 0L;
        this.readyLatencyAtFire = 0L;
    }

    public void setTargetRpm(double rpm, long nowMs) {
        int clamped = (rpm <= 0.0) ? 0 : (int) Math.round(Math.max(RPM_MIN, Math.min(RPM_MAX, rpm)));
        if (clamped != targetRpm) {
            targetRpm = clamped;
            targetSetMs = nowMs;
            readyStartMs = 0L;
            readyLatched = false;
            readyAtMs = 0L;
            readyLatencyMs = 0L;
            readyLatencyAtFire = 0L;
            commandFlywheel(clamped);
        }
    }

    public int getTargetRpm() {
        return targetRpm;
    }

    public void stop(long nowMs) {
        targetRpm = 0;
        commandFlywheel(0);
        targetSetMs = nowMs;
        readyLatched = false;
        readyStartMs = 0L;
        readyAtMs = 0L;
        readyLatencyMs = 0L;
        readyLatencyAtFire = 0L;
    }

    public boolean isReady(long nowMs) {
        return targetRpm > 0 && readyLatched && nowMs >= readyAtMs;
    }

    public boolean isLockedOut(long nowMs) {
        return nowMs < lockoutUntilMs;
    }

    public boolean isUnderLoad() {
        return targetRpm > 0;
    }

    public double getMeasuredRpm() {
        return filteredRpm;
    }

    public void update(long nowMs) {
        double measured = readRpm();
        filteredRpm = (EMA_ALPHA * measured) + ((1.0 - EMA_ALPHA) * filteredRpm);
        if (Double.isNaN(filteredRpm)) {
            filteredRpm = measured;
        }
        maintainReadyState(nowMs);
        monitorShotWindow(nowMs);
        serviceIntake();
        serviceLift(nowMs);
    }

    public boolean fire(long nowMs) {
        if (!isReady(nowMs) || isLockedOut(nowMs)) {
            return false;
        }
        openLift();
        pulseIntake();
        fireCommandMs = nowMs;
        rpmSnapshotAtFire = filteredRpm;
        shotDetected = false;
        pendingShot = null;
        readyLatencyAtFire = readyLatencyMs;
        readyLatched = false;
        readyStartMs = 0L;
        readyAtMs = 0L;
        readyLatencyMs = 0L;
        liftCloseAtMs = nowMs + LIFT_HOLD_MS;
        return true;
    }

    public ShotMetrics pollShotMetrics() {
        ShotMetrics metrics = pendingShot;
        pendingShot = null;
        return metrics;
    }

    private void maintainReadyState(long nowMs) {
        if (targetRpm <= 0) {
            readyLatched = false;
            readyStartMs = 0L;
            readyLatencyMs = 0L;
            readyLatencyAtFire = 0L;
            return;
        }
        double error = Math.abs(filteredRpm - targetRpm);
        if (error <= READY_TOL_RPM) {
            if (readyStartMs == 0L) {
                readyStartMs = nowMs;
            }
            if (!readyLatched && nowMs - readyStartMs >= READY_SETTLE_MS) {
                readyLatched = true;
                readyAtMs = nowMs;
                if (targetSetMs > 0L) {
                    long latency = nowMs - targetSetMs;
                    readyLatencyMs = Math.max(0L, latency);
                } else {
                    readyLatencyMs = 0L;
                }
            }
        } else {
            readyStartMs = 0L;
            readyLatched = false;
            readyAtMs = 0L;
            readyLatencyMs = 0L;
            readyLatencyAtFire = 0L;
        }
    }

    private void monitorShotWindow(long nowMs) {
        if (fireCommandMs == 0L) {
            return;
        }
        if (!shotDetected) {
            double drop = rpmSnapshotAtFire - filteredRpm;
            if (drop >= DIP_THRESHOLD_RPM) {
                shotDetected = true;
                lockoutUntilMs = nowMs + LOCKOUT_MS;
                pendingShot = new ShotMetrics(rpmSnapshotAtFire, readyLatencyAtFire, fireCommandMs);
                fireCommandMs = 0L;
                readyLatencyAtFire = 0L;
                return;
            }
            if (nowMs - fireCommandMs > DIP_WINDOW_MS) {
                // Timeout: treat as fired even if dip not detected to avoid stalling the loop.
                shotDetected = true;
                lockoutUntilMs = nowMs + LOCKOUT_MS;
                pendingShot = new ShotMetrics(rpmSnapshotAtFire, readyLatencyAtFire, fireCommandMs);
                fireCommandMs = 0L;
                readyLatencyAtFire = 0L;
            }
        }
    }

    private void commandFlywheel(int rpm) {
        if (flywheel == null) {
            return;
        }
        double ticksPerSecond = rpm * TICKS_PER_REV / 60.0;
        try {
            flywheel.setVelocity(ticksPerSecond);
        } catch (Exception ignored) {
        }
    }

    private double readRpm() {
        if (flywheel == null) {
            return 0.0;
        }
        try {
            double velocity = flywheel.getVelocity();
            return (velocity / TICKS_PER_REV) * 60.0;
        } catch (Exception e) {
            return 0.0;
        }
    }

    private void pulseIntake() {
        if (intake == null) {
            return;
        }
        try {
            intake.setPower(1.0);
            intakePulseEndNs = System.nanoTime() + INTAKE_PULSE_NS;
        } catch (Exception ignored) {
        }
    }

    private void serviceIntake() {
        if (intake == null) {
            return;
        }
        if (intakePulseEndNs > 0L && System.nanoTime() >= intakePulseEndNs) {
            try {
                intake.setPower(0.0);
            } catch (Exception ignored) {
            }
            intakePulseEndNs = 0L;
        }
    }

    private void openLift() {
        if (lift == null) {
            return;
        }
        try {
            lift.setPosition(BjornConstants.Servos.LIFT_RAISED);
        } catch (Exception ignored) {
        }
    }

    private void closeLift() {
        if (lift == null) {
            return;
        }
        try {
            lift.setPosition(BjornConstants.Servos.LIFT_LOWERED);
        } catch (Exception ignored) {
        }
    }

    private void serviceLift(long nowMs) {
        if (liftCloseAtMs > 0L && nowMs >= liftCloseAtMs) {
            closeLift();
            liftCloseAtMs = 0L;
        }
    }
}
