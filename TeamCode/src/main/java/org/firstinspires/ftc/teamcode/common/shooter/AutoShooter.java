package org.firstinspires.ftc.teamcode.common.shooter;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.common.BjornConstants;
import org.firstinspires.ftc.teamcode.common.BjornHardware;

public class AutoShooter {
    private final BjornHardware hardware;
    private int targetRpm = 0;
    private double filteredRpm = 0.0;
    private static final double EMA_ALPHA = 0.2;
    private static final double TICKS_PER_REV = 28.0;
    private static final double READY_TOL_RPM = 100.0;

    public AutoShooter(BjornHardware hardware) {
        this.hardware = hardware;
    }

    public void setTargetRpm(int rpm) {
        // Clamp to safe range if non-zero
        if (rpm > 0) {
            this.targetRpm = Math.max((int) BjornConstants.Power.SHOOTER_MIN_RPM,
                    Math.min(rpm, (int) BjornConstants.Power.SHOOTER_MAX_RPM));
        } else {
            this.targetRpm = 0;
        }
    }

    public void setIntakePower(boolean on) {
        if (hardware.intake != null) {
            hardware.intake.setPower(on ? 1.0 : 0.0);
        }
    }

    public boolean isReady() {
        if (targetRpm == 0)
            return false;
        return Math.abs(filteredRpm - targetRpm) < READY_TOL_RPM;
    }

    public void update() {
        // 1. Calculate RPM
        double measured = readRpm();
        filteredRpm = (EMA_ALPHA * measured) + ((1.0 - EMA_ALPHA) * filteredRpm);

        // 2. Battery Compensation
        int commandRpm = targetRpm;
        if (targetRpm > 0 && hardware.batterySensor != null) {
            double voltage = hardware.getBatteryVoltage();
            double nominal = BjornConstants.Power.NOMINAL_BATT_V;
            // Simple Feedforward compensation: if voltage drops, increase RPM target
            // slightly to maintain speed
            // This is a basic P-like adjustment based on voltage sag
            double sag = Math.max(0, nominal - voltage);
            // K_V is defined in constants, usually 0 but allows tuning
            commandRpm += (int) (sag * BjornConstants.Power.SHOOTER_K_V_RPM);
        }

        // 3. Set Motor Powers/Velocity
        setFlywheelRpm(commandRpm);

        // 4. Handle Ready State (LEDs + Auto Grip)
        boolean ready = isReady();
        updateLeds(ready);

        if (ready) {
            // Auto Grip Feed
            if (hardware.grip1 != null)
                hardware.grip1.setPower(1.0);
            if (hardware.grip2 != null)
                hardware.grip2.setPower(1.0);
        } else {
            // Stop grips if not ready (unless intake overrides, but AutoShooter owns this
            // now for Auto)
            // In Auto, usually we want to stop feeding if RPM drops.
            if (hardware.grip1 != null)
                hardware.grip1.setPower(0.0);
            if (hardware.grip2 != null)
                hardware.grip2.setPower(0.0);
        }
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

    private void updateLeds(boolean ready) {
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
