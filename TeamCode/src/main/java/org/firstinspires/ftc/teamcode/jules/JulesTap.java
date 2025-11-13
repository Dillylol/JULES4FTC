package org.firstinspires.ftc.teamcode.jules;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.pedropathing.follower.Follower; // For odometry pose

import org.firstinspires.ftc.teamcode.jules.telemetry.JulesDataOrganizer;

import java.util.List;
import java.util.ArrayList;

public class JulesTap {
    private final ElapsedTime clock = new ElapsedTime();
    private final DcMotorEx[] motors;
    private final VoltageSensor[] voltageSensors;
    private final double ticksPerRev;

    // These are no longer final so they can be updated live
    private double wheelDiameterIn;
    private double gearRatio;

    private final JulesDataOrganizer dataOrganizer = JulesDataOrganizer.getInstance();

    private volatile double lastCmd = 0.0;

    // --- Main Constructor ---
    public JulesTap(double ticksPerRev, double wheelDiameterIn, double gearRatio,
                    VoltageSensor battery, DcMotorEx... sampleMotors) {
        this(ticksPerRev, wheelDiameterIn, gearRatio,
                battery != null ? new VoltageSensor[]{battery} : new VoltageSensor[0],
                sampleMotors);
    }

    public JulesTap(double ticksPerRev, double wheelDiameterIn, double gearRatio,
                    VoltageSensor battery, List<DcMotorEx> sampleMotors) {
        this(ticksPerRev, wheelDiameterIn, gearRatio, battery, sampleMotors.toArray(new DcMotorEx[0]));
    }

    public JulesTap(double ticksPerRev, double wheelDiameterIn, double gearRatio,
                    Iterable<VoltageSensor> batteries, DcMotorEx... sampleMotors) {
        this(ticksPerRev, wheelDiameterIn, gearRatio, toArray(batteries), sampleMotors);
    }

    public JulesTap(double ticksPerRev, double wheelDiameterIn, double gearRatio,
                    Iterable<VoltageSensor> batteries, List<DcMotorEx> sampleMotors) {
        this(ticksPerRev, wheelDiameterIn, gearRatio, batteries, sampleMotors.toArray(new DcMotorEx[0]));
    }

    private JulesTap(double ticksPerRev, double wheelDiameterIn, double gearRatio,
                     VoltageSensor[] batteries, DcMotorEx... sampleMotors) {
        this.ticksPerRev = ticksPerRev;
        this.voltageSensors = batteries != null ? batteries : new VoltageSensor[0];
        this.motors = sampleMotors;
        updateConstants(wheelDiameterIn, gearRatio);
        clock.reset();
    }

    /**
     * Updates the physical constants used for velocity calculations.
     * Call this in your loop to ensure the tap is using the latest tuned values.
     */
    public void updateConstants(double wheelDiameterIn, double gearRatio) {
        this.wheelDiameterIn = wheelDiameterIn;
        this.gearRatio = gearRatio;
    }

    /** Call this right where you command drive power so the log matches the command. */
    public void setLastCmd(double cmdPower) { this.lastCmd = clamp(cmdPower, -1.0, 1.0); }

    /**
     * Samples all robot metrics.
     * This is the primary data collection method.
     * @param imu The robot's IMU for orientation and angular velocity.
     * @param follower The Pedro Pathing Follower for odometry pose.
     * @return A Metrics object populated with the latest data.
     */
    public Metrics sample(IMU imu, Follower follower) {
        Metrics m = new Metrics();
        m.t = clock.seconds();
        m.cmdPower = lastCmd;
        m.velIPS = encodersToIPS();
        double battery = safeVoltage();
        m.batteryV = Double.isFinite(battery) ? battery : m.batteryV;
        reportBatteryVoltage(battery);

        // --- NEW: Add Full Odometry and IMU Data ---
        if (follower != null) {
            m.x = follower.getPose().getX();
            m.y = follower.getPose().getY();
            m.heading = follower.getPose().getHeading();
        }

        if (imu != null) {
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
            m.headingDeg = orientation.getYaw(AngleUnit.DEGREES);
            m.pitch = orientation.getPitch(AngleUnit.DEGREES);
            m.roll = orientation.getRoll(AngleUnit.DEGREES);
            m.yawRate = angularVelocity.zRotationRate;
            m.pitchRate = angularVelocity.xRotationRate;
            m.rollRate = angularVelocity.yRotationRate;
        }
        return m;
    }

    /** Overloaded sample method for tests that don't need IMU/Odo. */
    public Metrics sample(double headingDeg) {
        Metrics m = new Metrics();
        m.t          = clock.seconds();
        m.cmdPower   = lastCmd;
        m.velIPS     = encodersToIPS();
        m.headingDeg = headingDeg;
        double battery = safeVoltage();
        m.batteryV   = Double.isFinite(battery) ? battery : m.batteryV;
        reportBatteryVoltage(battery);
        return m;
    }


    // ----- Helpers -----
    private double encodersToIPS() {
        // This calculation now uses the updatable member variables
        double tps = 0.0;
        for (DcMotorEx m : motors) {
            tps += m.getVelocity();
        }
        tps /= Math.max(1, motors.length);
        double inchesPerTick = (Math.PI * this.wheelDiameterIn) / (this.ticksPerRev * this.gearRatio);
        return tps * inchesPerTick;
    }

    private double safeVoltage() {
        double best = Double.NaN;
        for (VoltageSensor sensor : voltageSensors) {
            if (sensor == null) {
                continue;
            }
            try {
                double reading = sensor.getVoltage();
                if (Double.isFinite(reading)) {
                    if (Double.isNaN(best) || reading > best) {
                        best = reading;
                    }
                }
            } catch (Exception ignored) {
            }
        }
        return Double.isNaN(best) ? 12.0 : best;
    }

    private static double clamp(double v, double lo, double hi){ return Math.max(lo, Math.min(hi, v)); }

    private void reportBatteryVoltage(double voltage) {
        if (!Double.isFinite(voltage)) {
            return;
        }
        try {
            dataOrganizer.updateBatteryOverride(voltage);
        } catch (Exception ignored) {
        }
    }

    private static VoltageSensor[] toArray(Iterable<VoltageSensor> sensors) {
        if (sensors == null) {
            return new VoltageSensor[0];
        }
        List<VoltageSensor> list = new ArrayList<>();
        for (VoltageSensor sensor : sensors) {
            if (sensor != null) {
                list.add(sensor);
            }
        }
        return list.toArray(new VoltageSensor[0]);
    }
}