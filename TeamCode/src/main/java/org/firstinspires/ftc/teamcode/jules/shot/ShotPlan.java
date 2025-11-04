package org.firstinspires.ftc.teamcode.jules.shot;

/**
 * Immutable-ish data class describing the planner's guidance for a single shot.
 * The fields are mutable only where necessary so the state machine can "consume"
 * the fire_now directive without creating new objects every loop.
 */
public final class ShotPlan {
    private double rpmBias;
    private boolean fireNow;
    private boolean loiter;
    private long receivedAtMs;

    public ShotPlan(double rpmBias, boolean fireNow, boolean loiter) {
        this.rpmBias = rpmBias;
        this.fireNow = fireNow;
        this.loiter = loiter;
    }

    public static ShotPlan idle() {
        return new ShotPlan(0.0, false, false);
    }

    public double getRpmBias() {
        return rpmBias;
    }

    public void setRpmBias(double rpmBias) {
        this.rpmBias = rpmBias;
    }

    public boolean isFireNow() {
        return fireNow;
    }

    public boolean isLoiter() {
        return loiter;
    }

    public void setLoiter(boolean loiter) {
        this.loiter = loiter;
    }

    public long getReceivedAtMs() {
        return receivedAtMs;
    }

    public void setReceivedAtMs(long receivedAtMs) {
        this.receivedAtMs = receivedAtMs;
    }

    /**
     * Consume the fire_now flag so it only triggers the state machine once.
     */
    public void consumeFireNow() {
        this.fireNow = false;
    }

    public void setFireNow(boolean fireNow) {
        this.fireNow = fireNow;
    }

    public ShotPlan copy() {
        ShotPlan copy = new ShotPlan(rpmBias, fireNow, loiter);
        copy.setReceivedAtMs(receivedAtMs);
        return copy;
    }
}
