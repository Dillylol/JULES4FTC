package org.firstinspires.ftc.teamcode.jules.shot;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

/**
 * Simple PedroPathing wrapper that moves the robot to a single range/heading
 * solution relative to a fixed AprilTag target.
 */
public final class PedroShotNavigator {
    private final Follower follower;
    private final double tagXIn;
    private final double tagYIn;
    private final double approachAxisDeg;
    private final double positionTolIn;
    private final double headingTolDeg;

    private double commandedRangeIn;
    private Pose targetPose;

    public PedroShotNavigator(Follower follower,
                               double tagXIn,
                               double tagYIn,
                               double approachAxisDeg,
                               double positionTolIn,
                               double headingTolDeg) {
        this.follower = follower;
        this.tagXIn = tagXIn;
        this.tagYIn = tagYIn;
        this.approachAxisDeg = approachAxisDeg;
        this.positionTolIn = positionTolIn;
        this.headingTolDeg = headingTolDeg;
        this.commandedRangeIn = 0.0;
    }

    public void initialize(double rangeIn) {
        commandedRangeIn = rangeIn;
        targetPose = buildTargetPose(rangeIn);
        try {
            follower.setStartingPose(targetPose);
        } catch (Exception ignored) {
        }
    }

    public void update() {
        try {
            follower.update();
        } catch (Exception ignored) {
        }
    }

    public void driveToRangeAndAim(double rangeIn) {
        double clamped = Math.max(0.0, rangeIn);
        boolean targetChanged = targetPose == null || Math.abs(clamped - commandedRangeIn) > 0.25;
        commandedRangeIn = clamped;
        Pose desired = buildTargetPose(clamped);
        if (targetChanged) {
            targetPose = desired;
            Pose current = safePose();
            try {
                PathChain chain = follower.pathBuilder()
                        .addPath(new BezierLine(current, desired))
                        .setLinearHeadingInterpolation(current.getHeading(), desired.getHeading())
                        .build();
                follower.followPath(chain, true);
            } catch (Exception ignored) {
            }
        } else {
            targetPose = desired;
        }
    }

    public boolean atAimPose(double desiredRangeIn) {
        Pose pose = safePose();
        double rangeError = Math.abs(rangeToTag(pose) - desiredRangeIn);
        double headingError = angleDeltaDeg(headingToTagDeg(pose), Math.toDegrees(pose.getHeading()));
        boolean withinPose = rangeError <= positionTolIn && Math.abs(headingError) <= headingTolDeg;
        boolean followerDone = true;
        try {
            followerDone = follower.atParametricEnd();
        } catch (Exception ignored) {
        }
        return withinPose && followerDone;
    }

    public PoseSnapshot snapshot() {
        Pose pose = safePose();
        return new PoseSnapshot(pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
    }

    public double rangeToTag() {
        return rangeToTag(safePose());
    }

    public double headingToTagDeg() {
        return headingToTagDeg(safePose());
    }

    private Pose safePose() {
        try {
            Pose pose = follower.getPose();
            if (pose != null) {
                return pose;
            }
        } catch (Exception ignored) {
        }
        return targetPose != null ? targetPose : new Pose(tagXIn, tagYIn, 0.0);
    }

    private Pose buildTargetPose(double rangeIn) {
        double axisRad = Math.toRadians(approachAxisDeg);
        double headingRad = Math.toRadians(normalizeDeg(approachAxisDeg + 180.0));
        double x = tagXIn + Math.cos(axisRad) * rangeIn;
        double y = tagYIn + Math.sin(axisRad) * rangeIn;
        return new Pose(x, y, headingRad);
    }

    private double rangeToTag(Pose pose) {
        double dx = tagXIn - pose.getX();
        double dy = tagYIn - pose.getY();
        return Math.hypot(dx, dy);
    }

    private double headingToTagDeg(Pose pose) {
        double dx = tagXIn - pose.getX();
        double dy = tagYIn - pose.getY();
        double deg = Math.toDegrees(Math.atan2(dy, dx));
        return normalizeDeg(deg);
    }

    private static double normalizeDeg(double deg) {
        double d = deg;
        while (d <= -180) {
            d += 360;
        }
        while (d > 180) {
            d -= 360;
        }
        return d;
    }

    private static double angleDeltaDeg(double a, double b) {
        return normalizeDeg(a - b);
    }

    public static final class PoseSnapshot {
        public final double xIn;
        public final double yIn;
        public final double headingDeg;

        public PoseSnapshot(double xIn, double yIn, double headingDeg) {
            this.xIn = xIn;
            this.yIn = yIn;
            this.headingDeg = headingDeg;
        }
    }
}
