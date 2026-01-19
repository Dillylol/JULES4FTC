package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * A wrapper around DcMotorEx that applies a multiplier (gear ratio) to power and velocity commands.
 * This allows normalizing motor outputs so that a power of 1.0 represents a standard unit of effort
 * across different mechanisms.
 */
public class GearRatioMotor implements DcMotorEx {
    private final DcMotorEx delegate;
    private final double multiplier;

    public GearRatioMotor(DcMotorEx delegate, double multiplier) {
        this.delegate = delegate;
        this.multiplier = multiplier;
    }

    @Override
    public void setMotorEnable() {
        delegate.setMotorEnable();
    }

    @Override
    public void setMotorDisable() {
        delegate.setMotorDisable();
    }

    @Override
    public boolean isMotorEnabled() {
        return delegate.isMotorEnabled();
    }

    @Override
    public void setVelocity(double angularRate, AngleUnit unit) {
        delegate.setVelocity(angularRate * multiplier, unit);
    }

    @Override
    public double getVelocity(AngleUnit unit) {
        return delegate.getVelocity(unit) / multiplier; // Inverse for reading? Or just raw? Usually we want input scaled.
        // If we set velocity * 2, the motor spins 2x. Reading it back should probably reveal the "logical" velocity?
        // For simplicity in this context (normalizing INPUTS), let's assume we want to scale INPUTS.
        // However, if we want full transparency, we might usually just leave getters raw or scaled inversely.
        // Let's stick to scaling setter inputs primarily as requested ("let 1 be no multiplier and establish normality").
    }

    @Override
    public void setPIDCoefficients(RunMode mode, PIDCoefficients pidCoefficients) {
        delegate.setPIDCoefficients(mode, pidCoefficients);
    }

    @Override
    public void setPIDFCoefficients(RunMode mode, PIDFCoefficients pidfCoefficients) throws UnsupportedOperationException {
        delegate.setPIDFCoefficients(mode, pidfCoefficients);
    }

    @Override
    public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {
        delegate.setVelocityPIDFCoefficients(p, i, d, f);
    }

    @Override
    public void setPositionPIDFCoefficients(double p) {
        delegate.setPositionPIDFCoefficients(p);
    }

    @Override
    public PIDCoefficients getPIDCoefficients(RunMode mode) {
        return delegate.getPIDCoefficients(mode);
    }

    @Override
    public PIDFCoefficients getPIDFCoefficients(RunMode mode) {
        return delegate.getPIDFCoefficients(mode);
    }

    @Override
    public void setTargetPositionTolerance(int tolerance) {
        delegate.setTargetPositionTolerance(tolerance);
    }

    @Override
    public int getTargetPositionTolerance() {
        return delegate.getTargetPositionTolerance();
    }

    @Override
    public double getCurrent(CurrentUnit unit) {
        return delegate.getCurrent(unit);
    }

    @Override
    public double getCurrentAlert(CurrentUnit unit) {
        return delegate.getCurrentAlert(unit);
    }

    @Override
    public void setCurrentAlert(double current, CurrentUnit unit) {
        delegate.setCurrentAlert(current, unit);
    }

    @Override
    public boolean isOverCurrent() {
        return delegate.isOverCurrent();
    }

    @Override
    public MotorConfigurationType getMotorType() {
        return delegate.getMotorType();
    }

    @Override
    public void setMotorType(MotorConfigurationType motorType) {
        delegate.setMotorType(motorType);
    }

    @Override
    public DcMotorController getController() {
        return delegate.getController();
    }

    @Override
    public int getPortNumber() {
        return delegate.getPortNumber();
    }

    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
        delegate.setZeroPowerBehavior(zeroPowerBehavior);
    }

    @Override
    public ZeroPowerBehavior getZeroPowerBehavior() {
        return delegate.getZeroPowerBehavior();
    }

    @Override
    public void setPowerFloat() {
        delegate.setPowerFloat();
    }

    @Override
    public boolean getPowerFloat() {
        return delegate.getPowerFloat();
    }

    @Override
    public void setTargetPosition(int position) {
        delegate.setTargetPosition(position);
    }

    @Override
    public int getTargetPosition() {
        return delegate.getTargetPosition();
    }

    @Override
    public boolean isBusy() {
        return delegate.isBusy();
    }

    @Override
    public int getCurrentPosition() {
        return delegate.getCurrentPosition();
    }

    @Override
    public void setMode(RunMode mode) {
        delegate.setMode(mode);
    }

    @Override
    public RunMode getMode() {
        return delegate.getMode();
    }

    @Override
    public void setDirection(Direction direction) {
        delegate.setDirection(direction);
    }

    @Override
    public Direction getDirection() {
        return delegate.getDirection();
    }

    @Override
    public void setPower(double power) {
        delegate.setPower(power * multiplier);
    }

    @Override
    public double getPower() {
        return delegate.getPower() / multiplier; // Return logical power
    }

    @Override
    public Manufacturer getManufacturer() {
        return delegate.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return delegate.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return delegate.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return delegate.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        delegate.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
        delegate.close();
    }

    @Override
    public void setVelocity(double angularRate) {
        delegate.setVelocity(angularRate * multiplier);
    }

    @Override
    public double getVelocity() {
        return delegate.getVelocity() / multiplier;
    }
}
