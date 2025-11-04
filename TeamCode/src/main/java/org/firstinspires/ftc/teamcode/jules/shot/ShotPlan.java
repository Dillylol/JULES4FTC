package org.firstinspires.ftc.teamcode.jules.shot;

import androidx.annotation.Nullable;

/**
 * Immutable container describing a planner command for a single-shot attempt.
 */
public final class ShotPlan {

    public final String cmdId;
    public final String trialId;
    public final double rangeDeltaIn;
    public final boolean loiter;
    public final double headingOffsetDeg;
    public final long sentAtMs;
    public final long validUntilMs;
    public final long receivedAtMs;

    private final Double rpmBias;
    private final Double rpmTargetAbs;

    ShotPlan(String cmdId,
             String trialId,
             @Nullable Double rpmBias,
             @Nullable Double rpmTargetAbs,
             double rangeDeltaIn,
             boolean loiter,
             double headingOffsetDeg,
             long sentAtMs,
             long validUntilMs,
             long receivedAtMs) {
        this.cmdId = cmdId;
        this.trialId = trialId;
        this.rpmBias = rpmBias;
        this.rpmTargetAbs = rpmTargetAbs;
        this.rangeDeltaIn = rangeDeltaIn;
        this.loiter = loiter;
        this.headingOffsetDeg = headingOffsetDeg;
        this.sentAtMs = sentAtMs;
        this.validUntilMs = validUntilMs;
        this.receivedAtMs = receivedAtMs;
    }

    /** Returns {@code true} when the planner provided an absolute RPM target. */
    public boolean hasAbsoluteTarget() {
        return rpmTargetAbs != null;
    }

    /** Absolute RPM target supplied by the planner, or {@code null} when bias mode is requested. */
    @Nullable
    public Double getRpmTargetAbs() {
        return rpmTargetAbs;
    }

    /** Planner-provided bias to apply to the base RPM model, or {@code null} when absent. */
    @Nullable
    public Double getRpmBias() {
        return rpmBias;
    }
}
