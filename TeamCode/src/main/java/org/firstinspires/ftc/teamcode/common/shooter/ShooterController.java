package org.firstinspires.ftc.teamcode.common.shooter;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.common.BjornConstants;
import org.firstinspires.ftc.teamcode.common.BjornHardware;

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
    private static final double READY_TOL_RPM = 150.0;
    private static final long READY_SETTLE_MS = 300L;
    private static final double DIP_THRESHOLD_RPM = 50.0;
    private static final long DIP_WINDOW_MS = 220L;
    private static final long LOCKOUT_MS = 700L;
    private static final long BOOT_HOLD_MS = 350L;
    private static final long INTAKE_PULSE_NS = 200_000_000L;
    private static final double RPM_MIN = 1200.0;
    private static final double RPM_MAX = 3000.0;
    private static final double EMA_ALPHA = 0.2;
    private static final long RAMP_DURATION_MS = 3000L; // 3 seconds (tunable)
    private static final double POWER_CAP_FRACTION = 0.8; // 80% of RPM_MAX

    private final DcMotorEx flywheel;
    private final DcMotorEx flywheelSecondary;
    private final DcMotorEx intake;
    private final BjornHardware hardware; // Use BjornHardware for boot control
    @Nullable
    private final VoltageSensor vSensor;

    private int targetRpm;
    private int commandedRpm = 0;
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

    private long bootStowAtMs;
    private long intakePulseEndNs;
    private int rampStartRpm = 0;
    private boolean rampActive = false;

    public ShooterController(@Nullable DcMotorEx flywheel,
            @Nullable DcMotorEx flywheelSecondary,
            @Nullable DcMotorEx intake,
            @Nullable BjornHardware hardware) {
        this(flywheel, flywheelSecondary, intake, hardware, null);
    }

    public ShooterController(@Nullable DcMotorEx flywheel,
            @Nullable DcMotorEx flywheelSecondary,
            @Nullable DcMotorEx intake,
            @Nullable BjornHardware hardware,
            @Nullable VoltageSensor vSensor) {
        this.flywheel = flywheel;
        this.flywheelSecondary = flywheelSecondary;
        this.intake = intake;
        this.hardware = hardware;
        this.vSensor = vSensor;
        this.targetRpm = 0;
        this.filteredRpm = 0.0;
        this.readyLatencyMs = 0L;
        this.readyLatencyAtFire = 0L;
    }

    public void setTargetRpm(double rpm, long nowMs) {
        double cappedRequest = Math.min(rpm, RPM_MAX * POWER_CAP_FRACTION);
        int base = (cappedRequest <= 0.0)
                ? 0
                : (int) Math.round(Math.max(RPM_MIN, Math.min(RPM_MAX, cappedRequest)));
        int compensated = applySoftCap(applyBatteryCompensation(base));
        if (compensated != targetRpm) {
            targetRpm = compensated;
            targetSetMs = nowMs;
            readyStartMs = 0L;
            readyLatched = false;
            readyAtMs = 0L;
            readyLatencyMs = 0L;
            readyLatencyAtFire = 0L;
            rampStartRpm = commandedRpm;
            rampActive = compensated > 0;
        }
    }

    public int getTargetRpm() {
        return targetRpm;
    }

    public void stop(long nowMs) {
        targetRpm = 0;
        commandedRpm = 0;
        commandFlywheel(0);
        targetSetMs = nowMs;
        readyLatched = false;
        readyStartMs = 0L;
        readyAtMs = 0L;
        readyLatencyMs = 0L;
        readyLatencyAtFire = 0L;
        rampActive = false;
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

    public double getRpmEstimate() {
        return filteredRpm;
    }

    private static final long BOOT_PULSE_MS = 350L; // Default pulse for auto
    public long autoPulseEndMs = 0L; // Made public/accessible or just keep private logic

    // Simplified Logic: Press -> Power 1.0. Release -> Power 0.0 (Let mechanism
    // return).

    public void update(long nowMs) {
        // ... (existing update logic)
        double measured = readRpm();
        filteredRpm = (EMA_ALPHA * measured) + ((1.0 - EMA_ALPHA) * filteredRpm);
        if (Double.isNaN(filteredRpm)) {
            filteredRpm = measured;
        }
        maintainReadyState(nowMs);
        monitorShotWindow(nowMs);
        updateFlywheelCommand(nowMs);
        serviceIntake();
        serviceBoot(nowMs);
    }

    private boolean pulsingIntake = false;

    public boolean fire(long nowMs) {
        return fire(nowMs, true);
    }

    // Auto Fire: Pulse for BOOT_PULSE_MS then cut power
    /*
     * // Boot Removed
     * public void fireBoot(long nowMs) {
     * if (hardware != null && hardware.boot != null) {
     * hardware.boot.setPower(1.0);
     * autoPulseEndMs = nowMs + BOOT_PULSE_MS;
     * }
     * }
     */

    // Manual: Direct Control
    // extending = true -> Power 1.0
    // extending = false -> Power 0.0
    // Manual: Bidirectional Control
    // Retract (Reverse/Unjam) takes priority over Extend.
    public void controlBoot(boolean extend, boolean retract, long nowMs) {
        // Placeholder: logic removed.
        // In future, this will control the Lift servo.
    }

    public void fireBootAuto(long nowMs) {
        // fireBoot(nowMs);
    }

    private void serviceBoot(long nowMs) {
        if (bootStowAtMs > 0L && nowMs >= bootStowAtMs) {
            stowBoot();
            bootStowAtMs = 0L;
        }
    }

    public boolean fire(long nowMs, boolean engageIntake) {
        if (!isReady(nowMs) || isLockedOut(nowMs)) {
            return false;
        }
        pulseFeed(engageIntake);
        kickBoot();
        bootStowAtMs = nowMs + BOOT_HOLD_MS;
        fireCommandMs = nowMs;
        rpmSnapshotAtFire = filteredRpm;
        shotDetected = false;
        pendingShot = null;
        readyLatencyAtFire = readyLatencyMs;
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
        if (flywheel == null && flywheelSecondary == null) {
            return;
        }
        double ticksPerSecond = rpm * TICKS_PER_REV / 60.0;
        setVelocitySafe(flywheel, ticksPerSecond);
        setVelocitySafe(flywheelSecondary, ticksPerSecond);
        if (rpm <= 0) {
            setPowerSafe(flywheel, 0.0);
            setPowerSafe(flywheelSecondary, 0.0);
        }
    }

    private double getBatteryVoltage() {
        if (vSensor == null) {
            return BjornConstants.Power.NOMINAL_BATT_V;
        }
        try {
            return vSensor.getVoltage();
        } catch (Exception ignored) {
            return BjornConstants.Power.NOMINAL_BATT_V;
        }
    }

    private int applyBatteryCompensation(int baseRpm) {
        if (baseRpm <= 0) {
            return 0;
        }

        double vNow = getBatteryVoltage();
        double dV = BjornConstants.Power.NOMINAL_BATT_V - vNow;
        double compensated = baseRpm + (BjornConstants.Power.SHOOTER_K_V_RPM * dV);
        if (compensated <= 0.0) {
            return 0;
        }
        compensated = Math.max(RPM_MIN, Math.min(RPM_MAX, compensated));
        return (int) Math.round(compensated);
    }

    private int applySoftCap(int rpm) {
        if (rpm <= 0) {
            return 0;
        }
        double softMax = RPM_MAX * POWER_CAP_FRACTION;
        return (int) Math.round(Math.min(rpm, softMax));
    }

    private void updateFlywheelCommand(long nowMs) {
        int desired = applySoftCap(targetRpm);
        if (desired <= 0) {
            commandedRpm = 0;
            rampActive = false;
            commandFlywheel(0);
            return;
        }

        if (!rampActive) {
            // Start ramp
            rampStartRpm = (int) filteredRpm; // Start from current speed
            rampActive = true;
            targetSetMs = nowMs; // Reset ramp timer
        }

        long dt = Math.max(0L, nowMs - targetSetMs);
        if (dt >= RAMP_DURATION_MS) {
            commandedRpm = desired;
            rampActive = false;
        } else {
            // S-Curve Ramping
            double x = Math.max(0.0, Math.min(1.0, (double) dt / RAMP_DURATION_MS));
            double s = x * x * (3.0 - 2.0 * x); // Smoothstep
            double blended = rampStartRpm + (desired - rampStartRpm) * s;
            commandedRpm = (int) Math.round(blended);
        }
        commandFlywheel(commandedRpm);
    }

    private double readRpm() {
        if (flywheel == null && flywheelSecondary == null) {
            return 0.0;
        }
        double v1 = safeVelocity(flywheel);
        double v2 = safeVelocity(flywheelSecondary);
        double avg = (flywheelSecondary != null) ? 0.5 * (v1 + v2) : v1;
        return (avg / TICKS_PER_REV) * 60.0;
    }

    private void pulseFeed(boolean engageIntake) {
        this.pulsingIntake = engageIntake;
        if (engageIntake && intake != null) {
            try {
                intake.setPower(1.0);
            } catch (Exception ignored) {
            }
        }
        // Pulse Grips always
        if (hardware != null) {
            try {
                if (hardware.grip1 != null)
                    hardware.grip1.setPower(1.0);
                if (hardware.grip2 != null)
                    hardware.grip2.setPower(1.0);
            } catch (Exception ignored) {
            }
        }
        intakePulseEndNs = System.nanoTime() + INTAKE_PULSE_NS;
    }

    public void setFeedManual(boolean active) {
        if (active) {
            if (hardware != null) {
                try {
                    if (hardware.grip1 != null)
                        hardware.grip1.setPower(1.0);
                    if (hardware.grip2 != null)
                        hardware.grip2.setPower(1.0);
                } catch (Exception ignored) {
                }
            }
        } else {
            // Only stop if NOT pulsing
            if (intakePulseEndNs == 0L || System.nanoTime() >= intakePulseEndNs) {
                if (hardware != null) {
                    try {
                        if (hardware.grip1 != null)
                            hardware.grip1.setPower(0.0);
                        if (hardware.grip2 != null)
                            hardware.grip2.setPower(0.0);
                    } catch (Exception ignored) {
                    }
                }
            }
        }
    }

    private void serviceIntake() {
        if (intakePulseEndNs > 0L && System.nanoTime() >= intakePulseEndNs) {
            // Stop Intake if we were pulsing it
            if (pulsingIntake && intake != null) {
                try {
                    intake.setPower(0.0);
                } catch (Exception ignored) {
                }
            }
            // Stop Grips (Only if we aren't manually feeding)
            // As discussed, simplified: Just stop them.
            if (hardware != null) {
                try {
                    if (hardware.grip1 != null)
                        hardware.grip1.setPower(0.0);
                    if (hardware.grip2 != null)
                        hardware.grip2.setPower(0.0);
                } catch (Exception ignored) {
                }
            }
            intakePulseEndNs = 0L;
            pulsingIntake = false;
        }
    }

    private static void setVelocitySafe(@Nullable DcMotorEx motor, double ticksPerSecond) {
        if (motor == null) {
            return;
        }
        try {
            motor.setVelocity(ticksPerSecond);
        } catch (Exception ignored) {
        }
    }

    private static double safeVelocity(@Nullable DcMotorEx motor) {
        if (motor == null) {
            return 0.0;
        }
        try {
            return motor.getVelocity();
        } catch (Exception e) {
            return 0.0;
        }
    }

    private static void setPowerSafe(@Nullable DcMotorEx motor, double power) {
        if (motor == null) {
            return;
        }
        try {
            motor.setPower(power);
        } catch (Exception ignored) {
        }
    }

    private void kickBoot() {
        if (hardware != null && hardware.boot != null) {
            try {
                hardware.boot.setPosition(BjornConstants.Servos.BOOT_KICK);
            } catch (Exception ignored) {
            }
        }
    }

    private void stowBoot() {
        if (hardware != null && hardware.boot != null) {
            try {
                hardware.boot.setPosition(BjornConstants.Servos.BOOT_STOW);
            } catch (Exception ignored) {
            }
        }
    }
}
