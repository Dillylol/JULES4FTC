package org.firstinspires.ftc.teamcode.steele27303;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.BjornConstants;
import org.firstinspires.ftc.teamcode.common.BjornHardware;
import org.firstinspires.ftc.teamcode.common.CameraConfig;
import org.firstinspires.ftc.teamcode.jules.cv.AprilTagCamera;
import org.firstinspires.ftc.teamcode.jules.shot.ShooterController;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;

@Autonomous(name = "BjornAutoBLUE", group = "Bjorn")
public class BlueTagShooterAuto extends OpMode {

    // ---------------- Hardware ----------------
    private Follower follower;
    private BjornHardware hardware;
    private ShooterController shooterController;
    private AprilTagCamera aprilTagCamera;
    private DistanceSensor tof;
    private Servo lift;

    // ---------------- State Machine ----------------
    private enum State {
        TO_SHOOT, SHOOT1,
        TO_GLANCE, DETECTING,
        TO_ROW_START,
        TO_ROW, GRAB_ROW, RETURN_ROW, SHOOT_ROW,
        TO_ALIGN1, TO_GRAB1, TO_ALIGN1_BACK, SHOOT_FALLBACK, // Fallback states
        SHOOT2,
        TO_PARK, DONE
    }

    private State state;

    // ---------------- Paths ----------------
    private PathChain toShoot, toGlance, toRowStart;
    private PathChain toRow, returnRow, toShootRow;
    private PathChain toAlign1, toGrab1, toAlign1Back, toPark;

    // ---------------- Obelisk Logic ----------------
    private static final long DETECTION_TIMEOUT_MS = 1000; // Time to wait for tag
    private long detectionStartMs = -1;
    private Queue<Integer> rowQueue = new LinkedList<>();
    private int currentRow = -1;

    // ---------------- Shooter/Intake Logic ----------------
    private static final double WHEEL_IDLE_RPM = 2000;
    private static final double WHEEL_MAX_RPM = 4000;
    private static final double WHEEL_MIN_RPM = 2000;
    private static final double M_RPM_PER_FT = 116.4042383594456;
    private static final double B_RPM_OFFSET = 2084.2966941424975;
    private static final double FINAL_RPM_OFFSET = 150.0;
    private static final double SENSOR_OFFSET_FT = 0.0;
    private static final int SCAN_SAMPLES = 15;

    private boolean scanStarted = false;
    private long scanDelayUntil = -1;
    private static final long SETTLE_BEFORE_SCAN_MS = 500L;
    private int scanLeft = 0;
    private final double[] scan = new double[SCAN_SAMPLES];

    private long shootPhaseStart = -1;
    private static final long SHOOT_WINDOW_MS = 10000L;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(BjornConstants.AutoPoses.Blue.START);

        hardware = BjornHardware.forAutonomous(hardwareMap);
        tof = hardware.frontTof;
        lift = hardware.lift;

        shooterController = new ShooterController(
                hardware.wheel,
                hardware.wheel2,
                hardware.intake,
                hardware,
                hardware.getVoltageSensor());

        aprilTagCamera = new AprilTagCamera();
        aprilTagCamera.start(hardwareMap, null);
        aprilTagCamera.setManualExposure(6, 250);

        buildStaticPaths();

        // Init mechanisms
        shooterController.stop(System.currentTimeMillis());
        if (lift != null)
            lift.setPosition(BjornConstants.Servos.LIFT_LOWERED);

        // Start motion
        follower.setMaxPower(0.7); // Align speed
        follower.followPath(toShoot);
        state = State.TO_SHOOT;
    }

    @Override
    public void start() {
        if (lift != null)
            lift.setPosition(BjornConstants.Servos.LIFT_LOWERED);
    }

    @Override
    public void loop() {
        follower.update();
        long nowMs = System.currentTimeMillis();
        shooterController.update(nowMs);

        switch (state) {
            case TO_SHOOT:
                if (follower.atParametricEnd()) {
                    beginShootPhase(nowMs);
                    state = State.SHOOT1;
                }
                break;

            case SHOOT1:
                if (runShootPhase(nowMs)) {
                    follower.setMaxPower(1.0);
                    follower.followPath(toGlance, true);
                    state = State.TO_GLANCE;
                }
                break;

            case TO_GLANCE:
                if (follower.atParametricEnd()) {
                    detectionStartMs = nowMs;
                    state = State.DETECTING;
                }
                break;

            case DETECTING:
                int detectedRow = detectRow();
                if (detectedRow != -1) {
                    // Found Obelisk!
                    telemetry.addData("Obelisk", "Found Row " + detectedRow);
                    buildRowQueue(detectedRow);
                    follower.followPath(toRowStart, true);
                    state = State.TO_ROW_START;
                } else if (nowMs - detectionStartMs > DETECTION_TIMEOUT_MS) {
                    // Timeout -> Fallback to Basic Bjorn
                    telemetry.addData("Obelisk", "Not Found - Fallback");
                    follower.followPath(toAlign1, true); // From Glance to Align1
                    state = State.TO_ALIGN1;
                }
                break;

            // ---------------- Obelisk Pathing ----------------
            case TO_ROW_START:
                if (follower.atParametricEnd()) {
                    if (rowQueue.isEmpty()) {
                        follower.followPath(toPark, true);
                        state = State.TO_PARK;
                    } else {
                        currentRow = rowQueue.poll();
                        toRow = buildRowPath(currentRow);
                        follower.followPath(toRow, true);
                        hardware.intake.setPower(0.7); // Intake ON
                        state = State.TO_ROW;
                    }
                }
                break;

            case TO_ROW:
                if (follower.atParametricEnd()) {
                    // At Row Target (Grabbed)
                    // TODO: Add wait/verify logic if needed
                    returnRow = buildReturnRowPath(currentRow);
                    follower.followPath(returnRow, true);
                    state = State.RETURN_ROW;
                }
                break;

            case RETURN_ROW:
                if (follower.atParametricEnd()) {
                    hardware.intake.setPower(0); // Intake OFF
                    toShootRow = buildShootRowPath();
                    follower.followPath(toShootRow, true);
                    state = State.SHOOT_ROW;
                }
                break;

            case SHOOT_ROW:
                if (follower.atParametricEnd()) {
                    beginShootPhase(nowMs);
                    state = State.GRAB_ROW; // Reusing state name for "Shooting Row"
                }
                break;

            case GRAB_ROW: // Actually SHOOTING
                if (runShootPhase(nowMs)) {
                    // Loop back to Row Start for next row
                    follower.followPath(toRowStart, true); // Re-use toRowStart logic?
                    // Actually we are at Shoot Zone. Path to Row Start is valid.
                    // But we need to rebuild it from Shoot Zone?
                    // toRowStart is defined from Glance Point.
                    // We need Shoot -> RowStart.
                    PathChain shootToRowStart = line(BjornConstants.AutoPoses.Blue.SHOOT_ZONE,
                            BjornConstants.AutoPoses.Blue.ROW_START);
                    follower.followPath(shootToRowStart, true);
                    state = State.TO_ROW_START;
                }
                break;

            // ---------------- Fallback (Basic Bjorn) ----------------
            case TO_ALIGN1:
                if (follower.atParametricEnd()) {
                    hardware.intake.setPower(0.7);
                    follower.followPath(toGrab1, true);
                    state = State.TO_GRAB1;
                }
                break;

            case TO_GRAB1:
                if (follower.atParametricEnd()) {
                    shooterController.setTargetRpm(WHEEL_IDLE_RPM, nowMs);
                    follower.followPath(toAlign1Back, true);
                    state = State.TO_ALIGN1_BACK;
                }
                break;

            case TO_ALIGN1_BACK:
                if (follower.atParametricEnd()) {
                    hardware.intake.setPower(0);
                    // Re-use shoot path logic or build new
                    PathChain alignToShoot = line(BjornConstants.AutoPoses.Blue.ALIGN1_BACK,
                            BjornConstants.AutoPoses.Blue.SHOOT_ZONE);
                    follower.followPath(alignToShoot, true);
                    state = State.SHOOT_FALLBACK;
                }
                break;

            case SHOOT_FALLBACK:
                if (follower.atParametricEnd()) {
                    beginShootPhase(nowMs);
                    state = State.SHOOT2;
                }
                break;

            case SHOOT2:
                if (runShootPhase(nowMs)) {
                    follower.followPath(toPark, true);
                    state = State.TO_PARK;
                }
                break;

            case TO_PARK:
                if (follower.atParametricEnd()) {
                    shooterController.stop(nowMs);
                    state = State.DONE;
                }
                break;

            case DONE:
                break;
        }

        telemetry.addData("State", state);
        telemetry.update();
    }

    // Fix enum
    // ... (I will fix this in the actual file content below)

    @Override
    public void stop() {
        if (shooterController != null)
            shooterController.stop(System.currentTimeMillis());
        if (aprilTagCamera != null)
            aprilTagCamera.close();
    }

    private void buildStaticPaths() {
        toShoot = line(BjornConstants.AutoPoses.Blue.START, BjornConstants.AutoPoses.Blue.SHOOT_ZONE);
        toGlance = line(BjornConstants.AutoPoses.Blue.SHOOT_ZONE, BjornConstants.AutoPoses.Blue.GLANCE_POINT);
        toRowStart = line(BjornConstants.AutoPoses.Blue.GLANCE_POINT, BjornConstants.AutoPoses.Blue.ROW_START);

        // Fallback Paths
        // From Glance to Align1? Or Shoot to Align1?
        // If fallback, we are at Glance Point.
        toAlign1 = line(BjornConstants.AutoPoses.Blue.GLANCE_POINT, BjornConstants.AutoPoses.Blue.ALIGN1);
        toGrab1 = line(BjornConstants.AutoPoses.Blue.ALIGN1, BjornConstants.AutoPoses.Blue.GRAB1);
        toAlign1Back = line(BjornConstants.AutoPoses.Blue.GRAB1, BjornConstants.AutoPoses.Blue.ALIGN1_BACK);
        toPark = line(BjornConstants.AutoPoses.Blue.SHOOT_ZONE, BjornConstants.AutoPoses.Blue.PARK);
    }

    private PathChain buildRowPath(int row) {
        Pose target;
        switch (row) {
            case 1:
                target = BjornConstants.AutoPoses.Blue.ROW1;
                break;
            case 2:
                target = BjornConstants.AutoPoses.Blue.ROW2;
                break;
            case 3:
                target = BjornConstants.AutoPoses.Blue.ROW3;
                break;
            default:
                target = BjornConstants.AutoPoses.Blue.ROW1;
                break;
        }
        return line(BjornConstants.AutoPoses.Blue.ROW_START, target);
    }

    private PathChain buildReturnRowPath(int row) {
        Pose target;
        switch (row) {
            case 1:
                target = BjornConstants.AutoPoses.Blue.ROW1;
                break;
            case 2:
                target = BjornConstants.AutoPoses.Blue.ROW2;
                break;
            case 3:
                target = BjornConstants.AutoPoses.Blue.ROW3;
                break;
            default:
                target = BjornConstants.AutoPoses.Blue.ROW1;
                break;
        }
        // Return to Row Start? Or Align1Back?
        // Let's return to Row Start to keep it simple and consistent
        return line(target, BjornConstants.AutoPoses.Blue.ROW_START);
    }

    private PathChain buildShootRowPath() {
        return line(BjornConstants.AutoPoses.Blue.ROW_START, BjornConstants.AutoPoses.Blue.SHOOT_ZONE);
    }

    private PathChain line(Pose a, Pose b) {
        return follower.pathBuilder()
                .addPath(new BezierLine(a, b))
                .setLinearHeadingInterpolation(a.getHeading(), b.getHeading())
                .build();
    }

    private int detectRow() {
        List<AprilTagCamera.TagObservation> tags = aprilTagCamera.pollDetections();
        for (AprilTagCamera.TagObservation tag : tags) {
            if (tag.id == CameraConfig.OBELISK_TAG_ID_21)
                return 1;
            if (tag.id == CameraConfig.OBELISK_TAG_ID_22)
                return 2;
            if (tag.id == CameraConfig.OBELISK_TAG_ID_23)
                return 3;
        }
        return -1;
    }

    private void buildRowQueue(int firstRow) {
        rowQueue.clear();
        rowQueue.add(firstRow);
        // Add others
        for (int i = 1; i <= 3; i++) {
            if (i != firstRow)
                rowQueue.add(i);
        }
    }

    private void beginShootPhase(long nowMs) {
        shootPhaseStart = nowMs;
        shooterController.setTargetRpm(WHEEL_IDLE_RPM, nowMs);
        if (lift != null)
            lift.setPosition(BjornConstants.Servos.LIFT_LOWERED);
        scanStarted = false;
        scanDelayUntil = nowMs + SETTLE_BEFORE_SCAN_MS;
        scanLeft = 0;
    }

    private boolean runShootPhase(long nowMs) {
        long elapsed = nowMs - shootPhaseStart;

        // 1. Start Scan
        if (!scanStarted && nowMs >= scanDelayUntil) {
            Arrays.fill(scan, 0.0);
            scanLeft = SCAN_SAMPLES;
            scanStarted = true;
        }

        // 2. Process Scan & Set RPM
        if (scanLeft > 0) {
            double inches = safeTofInches(tof);
            scan[SCAN_SAMPLES - scanLeft] = inches;
            scanLeft--;
            if (scanLeft == 0) {
                double distToF = median(scan) / 12.0 + SENSOR_OFFSET_FT;
                double distCV = -1.0;
                List<AprilTagCamera.TagObservation> tags = aprilTagCamera.pollDetections();
                for (AprilTagCamera.TagObservation tag : tags) {
                    if (tag.id == CameraConfig.BLUE_GOAL_TAG_ID) {
                        distCV = tag.z * 3.28084;
                        break;
                    }
                }
                double distPedro = Math.abs(BjornConstants.AutoPoses.Blue.SHOOT_ZONE.getY() - follower.getPose().getY())
                        / 12.0;

                double finalDist = distToF;
                if (distCV > 0) {
                    finalDist = (distToF * 0.4) + (distCV * 0.6);
                } else if (distToF < 0.5) {
                    finalDist = distPedro;
                }

                if (finalDist > 0.5 && finalDist <= 10.0) {
                    double dyn = clamp(M_RPM_PER_FT * finalDist + B_RPM_OFFSET, WHEEL_MIN_RPM, WHEEL_MAX_RPM);
                    shooterController.setTargetRpm(clamp(dyn + FINAL_RPM_OFFSET, WHEEL_MIN_RPM, WHEEL_MAX_RPM), nowMs);
                } else {
                    double fallback = (WHEEL_MIN_RPM + WHEEL_MAX_RPM) * 0.5;
                    shooterController.setTargetRpm(clamp(fallback + FINAL_RPM_OFFSET, WHEEL_MIN_RPM, WHEEL_MAX_RPM),
                            nowMs);
                }
            }
        }

        // 3. Fire if ready
        if (shooterController.isReady(nowMs) && !shooterController.isLockedOut(nowMs)) {
            shooterController.fire(nowMs);
        }

        // 4. End Condition
        if (elapsed >= SHOOT_WINDOW_MS) {
            shooterController.stop(nowMs);
            if (lift != null)
                lift.setPosition(BjornConstants.Servos.LIFT_LOWERED);
            return true;
        }
        return false;
    }

    private static double safeTofInches(DistanceSensor ds) {
        try {
            double d = ds.getDistance(DistanceUnit.INCH);
            return (Double.isNaN(d) || d <= 0) ? -1.0 : d;
        } catch (Exception e) {
            return -1.0;
        }
    }

    private static double median(double[] a) {
        double[] b = Arrays.copyOf(a, a.length);
        Arrays.sort(b);
        int n = b.length;
        return (n % 2 == 1) ? b[n / 2] : 0.5 * (b[n / 2 - 1] + b[n / 2]);
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
