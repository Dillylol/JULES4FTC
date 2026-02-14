package org.firstinspires.ftc.teamcode.jules.shot;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import androidx.annotation.Nullable;

/**
 * Converts single-shot range targets into Pedro follower commands.
 */
public final class PedroShotNavigator {

    private final Follower follower;
    private final Pose tagPose;
    private final double aimToleranceDeg;
    private final double rangeToleranceIn;
    private final double minRangeIn;
    private final double maxRangeIn;

    private double headingOffsetDeg;
    private double targetRangeIn;

    @Nullable
    private Pose lastCommandedPose;

    public PedroShotNavigator(Follower follower,
                              Pose tagPose,
                              double aimToleranceDeg,
                              double rangeToleranceIn,
                              double initialRangeIn,
                              double minRangeIn,
                              double maxRangeIn) {
        this.follower = follower;
        this.tagPose = tagPose;
        this.aimToleranceDeg = aimToleranceDeg;
        this.rangeToleranceIn = rangeToleranceIn;
        this.minRangeIn = Math.min(minRangeIn, maxRangeIn);
        this.maxRangeIn = Math.max(minRangeIn, maxRangeIn);
        this.targetRangeIn = clampRange(initialRangeIn);
        this.headingOffsetDeg = 0.0;
    }

    public void update() {
        if (follower != null) {
            follower.update();
        }
    }

    public void setHeadingOffsetDeg(double headingOffsetDeg) {
        this.headingOffsetDeg = headingOffsetDeg;
    }

    public void applyRangeDelta(double deltaIn) {
        targetRangeIn = clampRange(targetRangeIn + deltaIn);
    }

    public void setRangeTarget(double rangeIn) {
        targetRangeIn = clampRange(rangeIn);
    }

    public double getRangeTarget() {
        return targetRangeIn;
    }

    public void driveToTarget() {
        if (follower == null || tagPose == null) {
            return;
        }
        Pose current = follower.getPose();
        Pose target = computeTargetPose();
        if (current == null || target == null) {
            return;
        }
        if (!shouldHoldCurrentPath(target)) {
            PathChain chain = follower.pathBuilder()
                    .addPath(new BezierLine(current, target))
                    .setLinearHeadingInterpolation(current.getHeading(), target.getHeading())
                    .build();
            follower.followPath(chain, true);
            lastCommandedPose = target;
        }
    }

    public boolean isAimed() {
        if (follower == null) {
            return false;
        }
        Pose pose = follower.getPose();
        if (pose == null) {
            return false;
        }
        double distanceError = Math.abs(getRangeIn() - targetRangeIn);
        double desiredHeadingRad = desiredHeadingRad();
        double headingErrorDeg = Math.toDegrees(angleWrap(desiredHeadingRad - pose.getHeading()));
        double tagBearingError = Math.abs(headingErrorDeg);
        return distanceError <= rangeToleranceIn && tagBearingError <= aimToleranceDeg;
    }

    public ShotPlannerBridge.PoseSnapshot getPoseSnapshot() {
        if (follower == null) {
            return new ShotPlannerBridge.PoseSnapshot(0, 0, 0);
        }
        Pose pose = follower.getPose();
        if (pose == null) {
            return new ShotPlannerBridge.PoseSnapshot(0, 0, 0);
        }
        return new ShotPlannerBridge.PoseSnapshot(pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
    }

    public double getRangeIn() {
        if (follower == null || tagPose == null) {
            return targetRangeIn;
        }
        Pose pose = follower.getPose();
        if (pose == null) {
            return targetRangeIn;
        }
        double dx = tagPose.getX() - pose.getX();
        double dy = tagPose.getY() - pose.getY();
        return Math.hypot(dx, dy);
    }

    public double getHeadingToTagDeg() {
        if (follower == null || tagPose == null) {
            return 0.0;
        }
        Pose pose = follower.getPose();
        if (pose == null) {
            return 0.0;
        }
        double desired = desiredHeadingRad();
        double error = angleWrap(desired - pose.getHeading());
        return Math.toDegrees(error);
    }

    private boolean shouldHoldCurrentPath(Pose target) {
        if (lastCommandedPose == null) {
            return false;
        }
        double dx = lastCommandedPose.getX() - target.getX();
        double dy = lastCommandedPose.getY() - target.getY();
        double dHeading = Math.toDegrees(angleWrap(lastCommandedPose.getHeading() - target.getHeading()));
        return Math.hypot(dx, dy) <= rangeToleranceIn && Math.abs(dHeading) <= aimToleranceDeg;
    }

    @Nullable
    private Pose computeTargetPose() {
        if (tagPose == null) {
            return null;
        }
        double heading = tagPose.getHeading();
        double offsetHeadingRad = desiredHeadingRad();
        double targetX = tagPose.getX() - Math.cos(heading) * targetRangeIn;
        double targetY = tagPose.getY() - Math.sin(heading) * targetRangeIn;
        return new Pose(targetX, targetY, offsetHeadingRad);
    }

    private double desiredHeadingRad() {
        double baseHeading = (tagPose != null) ? tagPose.getHeading() : 0.0;
        return baseHeading + Math.toRadians(headingOffsetDeg);
    }

    private static double angleWrap(double angleRad) {
        while (angleRad > Math.PI) {
            angleRad -= 2.0 * Math.PI;
        }
        while (angleRad < -Math.PI) {
            angleRad += 2.0 * Math.PI;
        }
        return angleRad;
    }

    private double clampRange(double value) {
        double clamped = Math.max(minRangeIn, Math.min(maxRangeIn, value));
        return Math.max(0.0, clamped);
    }
}
