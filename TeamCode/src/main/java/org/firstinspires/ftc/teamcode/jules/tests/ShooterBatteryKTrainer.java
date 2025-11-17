package org.firstinspires.ftc.teamcode.jules.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.BjornConstants;
import org.firstinspires.ftc.teamcode.common.BjornHardware;
import org.firstinspires.ftc.teamcode.jules.shot.ShooterController;

/**
 * Linear OpMode that estimates the shooter battery compensation slope (K) by gathering
 * RPM vs. voltage samples while stepping through a set of targets.
 */
@Autonomous(name = "ShooterBatteryKTrainer", group = "JULES")
public final class ShooterBatteryKTrainer extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        BjornHardware robot = BjornHardware.forTeleOp(hardwareMap);
        ShooterController shooter = new ShooterController(
                robot.wheel,
                robot.wheel2,
                robot.intake,
                robot.lift,
                robot.getVoltageSensor()
        );

        final int[] rpmTargets = new int[]{2300, 2400, 2500, 2600};

        int n = 0;
        double sumDV = 0.0;
        double sumErr = 0.0;
        double sumDV2 = 0.0;
        double sumDVE = 0.0;

        telemetry.addLine("ShooterBatteryKTrainer ready");
        telemetry.addLine("Press PLAY to start, tap A to stop early");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            for (int target : rpmTargets) {
                if (!opModeIsActive()) {
                    break;
                }

                long now = System.currentTimeMillis();
                shooter.setTargetRpm(target, now);

                long settleStart = System.currentTimeMillis();
                while (opModeIsActive() && (System.currentTimeMillis() - settleStart < 1200L)) {
                    long loopNow = System.currentTimeMillis();
                    shooter.update(loopNow);
                    telemetry.addData("TargetRPM", target);
                    telemetry.addData("RPM", shooter.getRpmEstimate());
                    telemetry.addData("Vbatt", robot.getBatteryVoltage());
                    telemetry.update();
                    idle();
                }

                final int samples = 10;
                double sumBatt = 0.0;
                double sumRpm = 0.0;
                int collected = 0;
                for (int i = 0; i < samples && opModeIsActive(); i++) {
                    long loopNow = System.currentTimeMillis();
                    shooter.update(loopNow);
                    sumBatt += robot.getBatteryVoltage();
                    sumRpm += shooter.getRpmEstimate();
                    collected++;
                    sleep(30);
                }

                if (!opModeIsActive() || collected == 0) {
                    break;
                }

                double v = sumBatt / collected;
                double rpmAtFire = sumRpm / collected;
                double rpmError = rpmAtFire - target;
                double dV = BjornConstants.Power.NOMINAL_BATT_V - v;

                n++;
                sumDV += dV;
                sumErr += rpmError;
                sumDV2 += dV * dV;
                sumDVE += dV * rpmError;

                double kEstimate = 0.0;
                if (n > 1) {
                    double denom = (sumDV2 - (sumDV * sumDV) / n);
                    if (Math.abs(denom) > 1e-6) {
                        double slope = (sumDVE - (sumDV * sumErr) / n) / denom;
                        kEstimate = slope;
                    }
                }

                telemetry.addData("Samples", n);
                telemetry.addData("LastTarget", target);
                telemetry.addData("LastRPM", rpmAtFire);
                telemetry.addData("LastError", rpmError);
                telemetry.addData("LastV", v);
                telemetry.addData("K_estimate", kEstimate);
                telemetry.update();

                if (gamepad1.a) {
                    shooter.stop(System.currentTimeMillis());
                    return;
                }
            }
        }

        shooter.stop(System.currentTimeMillis());
    }
}
