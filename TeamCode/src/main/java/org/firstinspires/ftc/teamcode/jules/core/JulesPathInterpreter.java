package org.firstinspires.ftc.teamcode.jules.core;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Point;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.ArrayList;
import java.util.List;

/**
 * Interprets dynamic path commands from JULES App and builds Pedro PathChains
 * at runtime.
 * This eliminates the need for code rebuilds when iterating on autonomous
 * paths.
 *
 * Usage flow:
 * 1. path_start -> reset()
 * 2. path_add -> addSegment(type, x, y, ...) (called N times)
 * 3. path_follow -> execute(follower)
 */
public class JulesPathInterpreter {
    private static final String TAG = "JulesPathInterpreter";

    /** Represents one path segment before it is compiled. */
    public static class Segment {
        public final String type; // linear, tangential, constant
        public final double x;
        public final double y;
        public final double startHeading; // Linear only
        public final double endHeading; // Linear only
        public final double heading; // Tangential / Constant
        public final boolean reverse; // Tangential only

        public Segment(String type, double x, double y,
                double startHeading, double endHeading,
                double heading, boolean reverse) {
            this.type = type;
            this.x = x;
            this.y = y;
            this.startHeading = startHeading;
            this.endHeading = endHeading;
            this.heading = heading;
            this.reverse = reverse;
        }
    }

    private final List<Segment> segments = new ArrayList<>();
    private boolean pathActive = false;

    /** Reset builder for a new path chain. */
    public void reset() {
        segments.clear();
        pathActive = true;
        RobotLog.i(TAG, "Path interpreter reset - ready for segments");
    }

    /** Add a segment to the current path. */
    public void addSegment(String type, double x, double y,
            double startHeading, double endHeading,
            double heading, boolean reverse) {
        if (!pathActive) {
            // Auto-start if not explicitly started
            reset();
        }
        segments.add(new Segment(type, x, y, startHeading, endHeading, heading, reverse));
        RobotLog.i(TAG, "Added segment: " + type + " (" + x + ", " + y + ")");
    }

    /** Build and execute the accumulated path chain on the given follower. */
    public boolean execute(Follower follower) {
        if (follower == null) {
            RobotLog.w(TAG, "Cannot execute: Follower is null");
            return false;
        }
        if (segments.isEmpty()) {
            RobotLog.w(TAG, "Cannot execute: No segments defined");
            return false;
        }

        try {
            Pose currentPose = follower.getPose();
            if (currentPose == null) {
                currentPose = new Pose(0, 0, 0);
            }

            // Start building from current pose
            PathBuilder builder = follower.pathBuilder();
            Point lastPoint = new Point(currentPose.getX(), currentPose.getY());

            for (Segment seg : segments) {
                Point endPoint = new Point(seg.x, seg.y);
                builder.addPath(new BezierLine(lastPoint, endPoint));

                // Apply heading interpolation based on type
                switch (seg.type.toLowerCase()) {
                    case "linear":
                        builder.setLinearHeadingInterpolation(
                                Math.toRadians(seg.startHeading),
                                Math.toRadians(seg.endHeading));
                        break;
                    case "tangential":
                        builder.setTangentHeadingInterpolation();
                        if (seg.reverse) {
                            builder.setReversed(true);
                        }
                        break;
                    case "constant":
                        builder.setConstantHeadingInterpolation(Math.toRadians(seg.heading));
                        break;
                    default:
                        RobotLog.w(TAG, "Unknown segment type: " + seg.type);
                        builder.setTangentHeadingInterpolation();
                        break;
                }

                lastPoint = endPoint;
            }

            PathChain chain = builder.build();
            follower.followPath(chain, true);
            RobotLog.i(TAG, "Path chain built & following (" + segments.size() + " segments)");

            // Clear after execution
            segments.clear();
            pathActive = false;
            return true;

        } catch (Exception e) {
            RobotLog.e(TAG, "Path execution failed: " + e.getMessage());
            segments.clear();
            pathActive = false;
            return false;
        }
    }

    public int getSegmentCount() {
        return segments.size();
    }

    public boolean isActive() {
        return pathActive;
    }
}
