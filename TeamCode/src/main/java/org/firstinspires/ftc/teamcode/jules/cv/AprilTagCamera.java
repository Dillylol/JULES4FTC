package org.firstinspires.ftc.teamcode.jules.cv;

import androidx.annotation.Nullable;

import com.google.gson.JsonObject;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.jules.bridge.JulesStreamBus;
import org.firstinspires.ftc.teamcode.common.CameraConfig;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionPortal.CameraState;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.TimeUnit;

/**
 * AprilTag camera helper backed by the FTC VisionPortal AprilTagProcessor.
 */
public final class AprilTagCamera implements AutoCloseable {

    private static final String TAG = "AprilTagCamera";
    private static final double INCH_TO_METER = 0.0254;

    private final String webcamName;

    @Nullable
    private volatile VisionPortal visionPortal;
    @Nullable
    private volatile AprilTagProcessor aprilTagProcessor;
    @Nullable
    private JulesStreamBus streamBus;

    private volatile List<TagObservation> latestObservations = Collections.emptyList();
    @Nullable
    private volatile TagObservation latestGoalObservation;

    public AprilTagCamera() {
        this(CameraConfig.WEBCAM_NAME);
    }

    public AprilTagCamera(String webcamName) {
        this.webcamName = webcamName;
    }

    public synchronized void start(HardwareMap hardwareMap, @Nullable JulesStreamBus bus) {
        close();
        this.streamBus = bus;
        WebcamName camName;
        try {
            camName = hardwareMap.get(WebcamName.class, webcamName);
        } catch (Exception e) {
            RobotLog.ee(TAG, "Webcam %s missing: %s", webcamName, e.getMessage());
            return;
        }

        AprilTagProcessor processor = new AprilTagProcessor.Builder()
                .setLensIntrinsics(CameraConfig.FX, CameraConfig.FY, CameraConfig.CX, CameraConfig.CY)
                .build();
        try {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(camName)
                    .addProcessor(processor)
                    .build();
            aprilTagProcessor = processor;
        } catch (Exception e) {
            RobotLog.ee(TAG, "VisionPortal start failed: %s", e.getMessage());
            aprilTagProcessor = null;
            visionPortal = null;
        }
    }

    public List<TagObservation> pollDetections() {
        AprilTagProcessor processor = aprilTagProcessor;
        if (processor == null) {
            latestObservations = Collections.emptyList();
            latestGoalObservation = null;
            return latestObservations;
        }
        List<AprilTagDetection> detections = processor.getDetections();
        if (detections == null || detections.isEmpty()) {
            latestObservations = Collections.emptyList();
            latestGoalObservation = null;
            return latestObservations;
        }

        long now = System.currentTimeMillis();
        List<TagObservation> converted = new ArrayList<>(detections.size());
        TagObservation bestGoal = null;
        for (AprilTagDetection detection : detections) {
            if (detection == null || detection.ftcPose == null) {
                continue;
            }
            TagObservation obs = TagObservation.fromDetection(now, detection);
            converted.add(obs);
            if (obs.isGoal()) {
                if (bestGoal == null || Math.abs(obs.z) < Math.abs(bestGoal.z)) {
                    bestGoal = obs;
                }
            }
        }

        if (converted.isEmpty()) {
            latestObservations = Collections.emptyList();
            latestGoalObservation = null;
            return latestObservations;
        }

        List<TagObservation> snapshot = Collections.unmodifiableList(converted);
        latestObservations = snapshot;
        latestGoalObservation = bestGoal;
        return snapshot;
    }

    public void publishDetections() {
        JulesStreamBus bus = streamBus;
        List<TagObservation> observations = latestObservations;
        if (bus == null || observations.isEmpty()) {
            return;
        }
        for (TagObservation obs : observations) {
            bus.publishJsonLine(obs.toJson().toString());
        }
    }

    @Nullable
    public TagObservation getLatestGoalObservation() {
        return latestGoalObservation;
    }

    public int getLastDetectionCount() {
        return latestObservations.size();
    }

    public void setManualExposure(int exposureMs, int gain) {
        VisionPortal portal = visionPortal;
        if (portal == null || portal.getCameraState() != CameraState.STREAMING) {
            return;
        }
        ExposureControl exposureControl = portal.getCameraControl(ExposureControl.class);
        if (exposureControl != null) {
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
            }
            exposureControl.setExposure((long) exposureMs, TimeUnit.MILLISECONDS);
        }
        GainControl gainControl = portal.getCameraControl(GainControl.class);
        if (gainControl != null) {
            gainControl.setGain(gain);
        }
    }

    @Override
    public synchronized void close() {
        if (visionPortal != null) {
            try {
                visionPortal.close();
            } catch (Exception e) {
                RobotLog.ww(TAG, "VisionPortal close failed: %s", e.getMessage());
            }
            visionPortal = null;
        }
        aprilTagProcessor = null;
        latestObservations = Collections.emptyList();
        latestGoalObservation = null;
        streamBus = null;
    }

    public static final class TagObservation {
        public final long timestampMs;
        public final int id;
        public final String tagClass;
        public final double x;
        public final double y;
        public final double z;
        public final double yaw;
        public final double pitch;
        public final double roll;

        private TagObservation(long timestampMs,
                               int id,
                               String tagClass,
                               double x,
                               double y,
                               double z,
                               double yaw,
                               double pitch,
                               double roll) {
            this.timestampMs = timestampMs;
            this.id = id;
            this.tagClass = tagClass;
            this.x = x;
            this.y = y;
            this.z = z;
            this.yaw = yaw;
            this.pitch = pitch;
            this.roll = roll;
        }

        static TagObservation fromDetection(long ts, AprilTagDetection detection) {
            double x = detection.ftcPose.x * INCH_TO_METER;
            double y = detection.ftcPose.y * INCH_TO_METER;
            double z = detection.ftcPose.z * INCH_TO_METER;
            double yaw = detection.ftcPose.yaw;
            double pitch = detection.ftcPose.pitch;
            double roll = detection.ftcPose.roll;
            return new TagObservation(
                    ts,
                    detection.id,
                    CameraConfig.classify(detection.id),
                    x,
                    y,
                    z,
                    yaw,
                    pitch,
                    roll);
        }

        boolean isGoal() {
            return CameraConfig.CLASS_BLUE_GOAL.equals(tagClass)
                    || CameraConfig.CLASS_RED_GOAL.equals(tagClass);
        }

        JsonObject toJson() {
            JsonObject obj = new JsonObject();
            obj.addProperty("type", "apriltag");
            obj.addProperty("ts_ms", timestampMs);
            obj.addProperty("id", id);
            obj.addProperty("class", tagClass);
            obj.addProperty("x_m", x);
            obj.addProperty("y_m", y);
            obj.addProperty("z_m", z);
            obj.addProperty("yaw", yaw);
            obj.addProperty("pitch", pitch);
            obj.addProperty("roll", roll);
            return obj;
        }
    }
}
