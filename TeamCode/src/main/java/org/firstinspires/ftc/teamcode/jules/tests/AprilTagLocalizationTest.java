package org.firstinspires.ftc.teamcode.jules.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.jules.bridge.JulesBridgeManager;
import org.firstinspires.ftc.teamcode.jules.bridge.JulesStreamBus;
import org.firstinspires.ftc.teamcode.jules.cv.AprilTagCamera;
import org.firstinspires.ftc.teamcode.jules.cv.AprilTagCamera.TagObservation;

import java.util.Collections;
import java.util.List;
import java.util.Locale;

@TeleOp(name = "JULES: AprilTag Localization Test", group = "JULES")
public final class AprilTagLocalizationTest extends LinearOpMode {

    private AprilTagCamera aprilTagCamera;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing AprilTag camera...");
        telemetry.update();

        JulesStreamBus streamBus = null;
        JulesBridgeManager manager = JulesBridgeManager.getInstance();
        if (manager != null) {
            manager.prepare(hardwareMap.appContext);
            streamBus = manager.getStreamBus();
        }

        aprilTagCamera = new AprilTagCamera();
        try {
            aprilTagCamera.start(hardwareMap, streamBus);
        } catch (Exception e) {
            telemetry.addLine("Camera init failed: " + e.getMessage());
            telemetry.update();
        }

        telemetry.addLine("Ready. Press PLAY to stream detections.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            List<TagObservation> detections = aprilTagCamera != null
                    ? aprilTagCamera.pollDetections() : Collections.emptyList();
            if (aprilTagCamera != null) {
                aprilTagCamera.publishDetections();
            }

            telemetry.addData("detections", detections.size());
            int idx = 0;
            for (TagObservation obs : detections) {
                if (idx >= 3) {
                    break;
                }
                telemetry.addData(
                        String.format(Locale.US, "tag[%d]", idx),
                        String.format(Locale.US,
                                "id=%d cls=%s z=%.2f x=%.2f y=%.2f yaw=%.2f pitch=%.2f",
                                obs.id,
                                obs.tagClass,
                                obs.z,
                                obs.x,
                                obs.y,
                                obs.yaw,
                                obs.pitch));
                idx++;
            }
            telemetry.update();
            sleep(20);
        }
    }
}
