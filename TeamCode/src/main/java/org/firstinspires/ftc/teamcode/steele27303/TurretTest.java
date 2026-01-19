package org.firstinspires.ftc.teamcode.steele27303;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.common.BjornHardware;
import org.firstinspires.ftc.teamcode.common.BjornConstants;

@TeleOp(name = "TurretTest", group = "Test")
public class TurretTest extends LinearOpMode {

    private BjornHardware hardware;
    private static final double TURRET_POWER = 0.3; // Gentle speed for testing

    @Override
    public void runOpMode() {
        // Initialize Hardware
        hardware = BjornHardware.forTeleOp(hardwareMap);
        
        telemetry.addLine("Initialized TurretTest");
        telemetry.addLine("Use D-Pad Left/Right to rotate Turret");
        telemetry.addLine("Must stay between 0 and 180 degrees");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // 1. Run the Prediction Step (Critical for Belief State)
            hardware.updateEstimator();

            // 2. Get Current Belief
            double currentAngle = hardware.turretEstimator.getBelief();

            // 3. User Control with Soft Limits
            double power = 0.0;

            if (gamepad1.dpad_left) {
                // Moving towards positive? Or negative?
                // Let's assume Left = Positive (0 -> 180)
                if (currentAngle < 180.0) {
                    power = TURRET_POWER;
                }
            } else if (gamepad1.dpad_right) {
                // Right = Negative (180 -> 0)
                if (currentAngle > 0.0) {
                    power = -TURRET_POWER;
                }
            }
            
            // Apply Power
            if (hardware.turret != null) {
                hardware.turret.setPower(power);
            }

            // 4. Telemetry
            telemetry.addData("Turret Belief (Deg)", "%.2f", currentAngle);
            telemetry.addData("Turret Conflict", "%.4f", hardware.turretEstimator.getConflict());
            telemetry.addData("Estimator Uncertainty", "%.4f", hardware.turretEstimator.getUncertainty());
            telemetry.addData("Motor Power", "%.2f", power);
            
            // Note: In a real scenario, we would also call hardware.correctTurretBelief()
            // if we had camera data here. For this test, we are verifying Encoder Prediction.
            
            telemetry.update();
        }
    }
}
