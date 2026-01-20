package org.firstinspires.ftc.teamcode.camerastream;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.common.CameraConfig;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Mat;

import java.util.concurrent.atomic.AtomicReference;

/**
 * Test Camera Stream - Streams camera for viewing
 * 
 * This OpMode enables camera streaming for remote viewing.
 * Use Driver Station camera preview or Panels UI.
 */
@TeleOp(name = "Test Camera Stream", group = "Dev")
public class TestCameraStream extends OpMode {

    /**
     * Vision processor that captures frames for streaming
     */
    static class StreamProcessor implements VisionProcessor, CameraStreamSource {

        private final AtomicReference<Bitmap> lastFrame = new AtomicReference<>(
                Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

        @Override
        public void init(int width, int height, CameraCalibration calibration) {
            lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            Bitmap b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
            Utils.matToBitmap(frame, b);
            lastFrame.set(b);
            return null;
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
            // No custom drawing
        }

        @Override
        public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
            continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
        }
    }

    private final StreamProcessor processor = new StreamProcessor();
    private VisionPortal visionPortal;

    @Override
    public void init() {
        visionPortal = new VisionPortal.Builder()
                .addProcessor(processor)
                .setCamera(hardwareMap.get(WebcamName.class, CameraConfig.WEBCAM_NAME))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .build();

        // Note: For Panels integration, the VisionPortal's CameraStreamSource
        // will be auto-detected by Panels if the library is configured

        telemetry.addLine("Camera Stream Started");
        telemetry.addLine("View via DS Camera Preview or Panels");
        telemetry.update();
    }

    @Override
    public void loop() {
        telemetry.addData("Camera State", visionPortal.getCameraState());
        telemetry.update();
    }

    @Override
    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}
