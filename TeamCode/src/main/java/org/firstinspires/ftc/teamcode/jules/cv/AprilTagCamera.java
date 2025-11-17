package org.firstinspires.ftc.teamcode.jules.cv;

import android.graphics.Bitmap;
import android.util.Base64;

import androidx.annotation.Nullable;

import com.google.gson.JsonObject;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.CameraConfig;
import org.firstinspires.ftc.teamcode.jules.bridge.JulesStreamBus;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCamera.AsyncCameraOpenListener;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.io.ByteArrayOutputStream;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Shared AprilTag camera helper that wraps EasyOpenCV + OpenFTC pipeline and forwards
 * detections and video frames into the JULES stream bus.
 */
public final class AprilTagCamera implements AutoCloseable {

    private static final String TAG = "AprilTagCamera";
    private static final int STREAM_WIDTH = 800;
    private static final int STREAM_HEIGHT = 448;
    private static final long DETECTION_PUBLISH_INTERVAL_MS = 75L;
    private static final long VIDEO_PUBLISH_INTERVAL_MS = 250L;

    private final String webcamName;

    @Nullable
    private OpenCvCamera camera;
    @Nullable
    private AprilTagDetectionPipeline pipeline;
    @Nullable
    private JulesStreamBus streamBus;

    private volatile boolean opened;
    private volatile List<TagObservation> latestObservations = Collections.emptyList();
    @Nullable
    private volatile TagObservation latestGoalObservation;
    private volatile long lastDetectionPublishMs = 0L;
    private volatile long lastVideoPublishMs = 0L;

    public AprilTagCamera() {
        this(CameraConfig.WEBCAM_NAME);
    }

    public AprilTagCamera(String webcamName) {
        this.webcamName = webcamName;
    }

    public void start(HardwareMap hardwareMap, @Nullable JulesStreamBus bus) {
        this.streamBus = bus;
        WebcamName camName;
        try {
            camName = hardwareMap.get(WebcamName.class, webcamName);
        } catch (Exception e) {
            RobotLog.ee(TAG, "Webcam %s missing: %s", webcamName, e.getMessage());
            return;
        }

        int monitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId",
                "id",
                hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory.getInstance().createWebcam(camName, monitorViewId);
        pipeline = new AprilTagDetectionPipeline(
                CameraConfig.TAG_SIZE_METERS,
                CameraConfig.FX,
                CameraConfig.FY,
                CameraConfig.CX,
                CameraConfig.CY);
        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                opened = true;
                camera.startStreaming(STREAM_WIDTH, STREAM_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                RobotLog.ee(TAG, "Camera open failed: %d", errorCode);
            }
        });
    }

    public List<TagObservation> pollDetections() {
        AprilTagDetectionPipeline pipe = pipeline;
        if (pipe == null) {
            latestObservations = Collections.emptyList();
            latestGoalObservation = null;
            return latestObservations;
        }
        List<AprilTagDetection> detections = pipe.getLatestDetections();
        if (detections == null || detections.isEmpty()) {
            latestObservations = Collections.emptyList();
            latestGoalObservation = null;
            return latestObservations;
        }

        long now = System.currentTimeMillis();
        List<TagObservation> converted = new ArrayList<>(detections.size());
        TagObservation bestGoal = null;
        for (AprilTagDetection detection : detections) {
            TagObservation obs = TagObservation.fromDetection(now, detection);
            converted.add(obs);
            if (obs.isGoal()) {
                if (bestGoal == null || obs.z < bestGoal.z) {
                    bestGoal = obs;
                }
            }
        }
        latestObservations = converted;
        latestGoalObservation = bestGoal;
        return converted;
    }

    public void publishDetections() {
        JulesStreamBus bus = streamBus;
        if (bus == null || latestObservations.isEmpty()) {
            return;
        }
        long now = System.currentTimeMillis();
        if (now - lastDetectionPublishMs < DETECTION_PUBLISH_INTERVAL_MS) {
            return;
        }
        for (TagObservation obs : latestObservations) {
            bus.publishJsonLine(obs.toJson().toString());
        }
        lastDetectionPublishMs = now;
    }

    public void publishVideoFrame() {
        JulesStreamBus bus = streamBus;
        OpenCvCamera cam = camera;
        if (bus == null || cam == null || !opened) {
            return;
        }
        long now = System.currentTimeMillis();
        if (now - lastVideoPublishMs < VIDEO_PUBLISH_INTERVAL_MS) {
            return;
        }
        Bitmap bitmap;
        try {
            bitmap = cam.getFrameBitmap();
        } catch (Exception e) {
            RobotLog.ww(TAG, "Frame grab failed: %s", e.getMessage());
            return;
        }
        if (bitmap == null) {
            return;
        }
        try {
            ByteArrayOutputStream baos = new ByteArrayOutputStream();
            bitmap.compress(Bitmap.CompressFormat.JPEG, 55, baos);
            byte[] jpeg = baos.toByteArray();
            JsonObject packet = new JsonObject();
            packet.addProperty("type", "video_frame");
            packet.addProperty("ts_ms", now);
            packet.addProperty("jpeg_b64", Base64.encodeToString(jpeg, Base64.NO_WRAP));
            bus.publishJsonLine(packet.toString());
            lastVideoPublishMs = now;
        } catch (Exception e) {
            RobotLog.ee(TAG, "Failed to publish video frame: %s", e.getMessage());
        } finally {
            bitmap.recycle();
        }
    }

    @Nullable
    public TagObservation getLatestGoalObservation() {
        return latestGoalObservation;
    }

    public int getLastDetectionCount() {
        return latestObservations.size();
    }

    @Override
    public void close() {
        opened = false;
        if (camera != null) {
            try {
                camera.stopStreaming();
            } catch (Exception ignored) {
            }
            try {
                camera.closeCameraDevice();
            } catch (Exception ignored) {
            }
            camera = null;
        }
        pipeline = null;
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
            double x = detection.pose.x;
            double y = detection.pose.y;
            double z = detection.pose.z;
            double yaw = detection.pose.yaw;
            double pitch = detection.pose.pitch;
            double roll = detection.pose.roll;
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

