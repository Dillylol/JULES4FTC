// File: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/jules/Metrics.java
package org.firstinspires.ftc.teamcode.jules;

public final class Metrics {
    // Core Metrics
    public double t;          // seconds since init
    public double cmdPower;   // -1..1
    public double velIPS;     // inches/second
    public double batteryV;   // volts
    public String label;      // optional tag

    // Odometry
    public double x;          // inches
    public double y;          // inches
    public double heading;    // radians

    // Detailed IMU
    public double headingDeg; // degrees
    public double pitch;      // degrees
    public double roll;       // degrees
    public double yawRate;    // degrees/sec
    public double pitchRate;  // degrees/sec
    public double rollRate;   // degrees/sec
    public String jsonData;
}