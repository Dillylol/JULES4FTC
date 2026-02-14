package org.firstinspires.ftc.teamcode.jules.telemetry;

import android.content.Context;
import android.os.Build;

import com.google.gson.JsonElement;
import com.google.gson.JsonNull;
import com.google.gson.JsonObject;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.jules.bridge.util.GsonCompat;


import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.Map;
import java.util.Objects;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Aggregates hardware, gamepad, and telemetry data into JSON snapshots.
 */
public class JulesDataOrganizer {

    public enum OpModeState {
        INIT,
        RUNNING,
        STOPPED
    }

    private static final JulesDataOrganizer INSTANCE = new JulesDataOrganizer();

    public static JulesDataOrganizer getInstance() {
        return INSTANCE;
    }

    private final Object lock = new Object();

    private final Map<String, DcMotor> motors = new LinkedHashMap<>();
    private final Map<String, Servo> servos = new LinkedHashMap<>();
    private final Map<String, CRServo> crServos = new LinkedHashMap<>();
    private final Map<String, IMU> imus = new LinkedHashMap<>();
    private final Map<String, DistanceSensor> distanceSensors = new LinkedHashMap<>();
    private final Map<String, com.qualcomm.robotcore.hardware.ColorSensor> colorSensors = new LinkedHashMap<>();
    private final Map<String, TouchSensor> touchSensors = new LinkedHashMap<>();
    private final Map<String, AnalogInput> analogInputs = new LinkedHashMap<>();
    private final Map<String, DigitalChannel> digitalChannels = new LinkedHashMap<>();
    private final Set<VoltageSensor> voltageSensors = new LinkedHashSet<>();

    private final ConcurrentHashMap<String, String> telemetryCustom = new ConcurrentHashMap<>();

    private static final long BATTERY_OVERRIDE_STALE_MS = 2000L;

    private final AtomicLong sequence = new AtomicLong();

    private HardwareMap hardwareMap;
    private Gamepad gamepad1;
    private Gamepad gamepad2;
    private OpMode opMode;
    private OpModeState opModeState = OpModeState.INIT;
    private String sdkVersion = "UNKNOWN";
    private String appVersion = "UNKNOWN";
    private String deviceModel = Build.MODEL == null ? "UNKNOWN" : Build.MODEL;

    private JsonObject latestSnapshotData = new JsonObject();
    private JsonObject latestSnapshotMessage = null;
    private final AtomicReference<BatterySample> batteryOverride = new AtomicReference<>(BatterySample.EMPTY);

    private JulesDataOrganizer() {
    }

    public void bind(OpMode opMode, HardwareMap hardwareMap, Gamepad g1, Gamepad g2, Telemetry telemetry) {
        synchronized (lock) {
            this.opMode = opMode;
            this.hardwareMap = hardwareMap;
            this.gamepad1 = g1;
            this.gamepad2 = g2;
            discoverHardware();
            resolveVersions(hardwareMap);
        }
    }

    private void discoverHardware() {
        motors.clear();
        servos.clear();
        crServos.clear();
        imus.clear();
        distanceSensors.clear();
        colorSensors.clear();
        touchSensors.clear();
        analogInputs.clear();
        digitalChannels.clear();
        voltageSensors.clear();

        if (hardwareMap == null) {
            return;
        }

        for (DcMotor motor : hardwareMap.getAll(DcMotor.class)) {
            String name = getFirstName(motor);
            if (name != null) {
                motors.put(name, motor);
            }
        }

        for (Servo servo : hardwareMap.getAll(Servo.class)) {
            String name = getFirstName(servo);
            if (name != null) {
                servos.put(name, servo);
            }
        }

        for (CRServo servo : hardwareMap.getAll(CRServo.class)) {
            String name = getFirstName(servo);
            if (name != null) {
                crServos.put(name, servo);
            }
        }

        for (IMU imu : hardwareMap.getAll(IMU.class)) {
            String name = getFirstName(imu);
            if (name != null) {
                imus.put(name, imu);
            }
        }

        for (DistanceSensor sensor : hardwareMap.getAll(DistanceSensor.class)) {
            String name = getFirstName(sensor);
            if (name != null) {
                distanceSensors.put(name, sensor);
            }
        }

        for (com.qualcomm.robotcore.hardware.ColorSensor sensor : hardwareMap.getAll(com.qualcomm.robotcore.hardware.ColorSensor.class)) {
            String name = getFirstName(sensor);
            if (name != null) {
                colorSensors.put(name, sensor);
            }
        }

        for (TouchSensor sensor : hardwareMap.getAll(TouchSensor.class)) {
            String name = getFirstName(sensor);
            if (name != null) {
                touchSensors.put(name, sensor);
            }
        }

        for (AnalogInput input : hardwareMap.getAll(AnalogInput.class)) {
            String name = getFirstName(input);
            if (name != null) {
                analogInputs.put(name, input);
            }
        }

        for (DigitalChannel channel : hardwareMap.getAll(DigitalChannel.class)) {
            String name = getFirstName(channel);
            if (name != null) {
                digitalChannels.put(name, channel);
            }
        }

        for (VoltageSensor sensor : hardwareMap.getAll(VoltageSensor.class)) {
            voltageSensors.add(sensor);
        }
    }

    private void resolveVersions(HardwareMap map) {
        // SDK version via RobotLog (handle name differences)
        try {
            try {
                sdkVersion = (String) RobotLog.class.getMethod("getVersion").invoke(null);
            } catch (NoSuchMethodException e1) {
                sdkVersion = (String) RobotLog.class.getMethod("getVersionString").invoke(null);
            }
        } catch (Throwable t) {
            sdkVersion = "UNKNOWN";
        }

        // App version: try BuildConfig reflectively; fall back to PackageManager
        try {
            Class<?> bc = Class.forName("org.firstinspires.ftc.teamcode.BuildConfig");
            Object verName = bc.getField("VERSION_NAME").get(null);
            appVersion = String.valueOf(verName);
        } catch (Throwable t) {
            try {
                Context ctx = map != null ? map.appContext : null;
                if (ctx != null) {
                    appVersion = ctx.getPackageManager()
                            .getPackageInfo(ctx.getPackageName(), 0).versionName;
                } else {
                    appVersion = "UNKNOWN";
                }
            } catch (Throwable ignored) {
                appVersion = "UNKNOWN";
            }
        }
    }


    private String getFirstName(HardwareDevice device) {
        if (hardwareMap == null || device == null) {
            return null;
        }
        Set<String> names = hardwareMap.getNamesOf(device);
        if (names == null || names.isEmpty()) {
            return null;
        }
        return names.iterator().next();
    }

    public void setOpModeState(OpModeState state) {
        synchronized (lock) {
            opModeState = state;
            if (state == OpModeState.STOPPED) {
                batteryOverride.set(BatterySample.EMPTY);
            }
        }
    }

    public OpModeState getOpModeState() {
        synchronized (lock) {
            return opModeState;
        }
    }

    public void recordTelemetry(String key, Object value) {
        if (key == null) return;
        telemetryCustom.put(key, String.valueOf(value));
    }

    public void clearTelemetry() {
        telemetryCustom.clear();
    }

    public void removeTelemetryKey(String key) {
        if (key == null) {
            return;
        }
        telemetryCustom.remove(key);
    }

    public JsonObject buildHeartbeat() {
        JsonObject heartbeat = new JsonObject();
        long now = System.currentTimeMillis();
        heartbeat.addProperty("type", "heartbeat");
        heartbeat.addProperty("ts_ms", now);
        heartbeat.addProperty("seq", sequence.incrementAndGet());
        heartbeat.addProperty("uptime_ms", getOpModeUptimeMs());
        double battery = getBatteryVoltage();
        if (Double.isFinite(battery)) {
            heartbeat.addProperty("battery_v", round3(battery));
        }
        heartbeat.addProperty("active_opmode", getActiveOpModeName());
        return heartbeat;
    }

    public JsonObject buildSnapshot() {
        JsonObject data;
        long now = System.currentTimeMillis();
        synchronized (lock) {
            data = buildData(now);
            latestSnapshotData = GsonCompat.deepCopy(data);
        }

        JsonObject snapshot = new JsonObject();
        snapshot.addProperty("type", "snapshot");
        snapshot.addProperty("ts_ms", now);
        snapshot.addProperty("active_opmode", getActiveOpModeName());
        snapshot.add("data", data);
        synchronized (lock) {
            latestSnapshotMessage = GsonCompat.deepCopy(snapshot);
        }
        return snapshot;
    }

    public void updateBatteryOverride(double voltage) {
        if (!Double.isFinite(voltage)) {
            return;
        }
        batteryOverride.set(new BatterySample(round3(voltage), System.currentTimeMillis()));
    }

    public void clearBatteryOverride() {
        batteryOverride.set(BatterySample.EMPTY);
    }

    public JsonObject buildDiffSince(JsonObject lastData) {
        if (lastData == null) {
            return null;
        }
        long now = System.currentTimeMillis();
        JsonObject newData;
        synchronized (lock) {
            newData = buildData(now);
        }
        Map<String, JsonElement> previous = flattenJson(lastData, "");
        Map<String, JsonElement> current = flattenJson(newData, "");

        JsonObject patch = new JsonObject();
        for (Map.Entry<String, JsonElement> entry : current.entrySet()) {
            String path = entry.getKey();
            JsonElement currentValue = entry.getValue();
            JsonElement previousValue = previous.get(path);
            if (!Objects.equals(sanitize(previousValue), sanitize(currentValue))) {
                patch.add(path, currentValue);
            }
        }
        for (String path : previous.keySet()) {
            if (!current.containsKey(path)) {
                patch.add(path, JsonNull.INSTANCE);
            }
        }

        if (patch.entrySet().isEmpty()) {
            return null;
        }

        synchronized (lock) {
            latestSnapshotData = GsonCompat.deepCopy(newData);
        }

        JsonObject diff = new JsonObject();
        diff.addProperty("type", "diff");
        diff.addProperty("ts_ms", now);
        diff.addProperty("active_opmode", getActiveOpModeName());
        diff.add("patch", patch);
        return diff;
    }

    private JsonElement sanitize(JsonElement element) {
        if (element == null) {
            return JsonNull.INSTANCE;
        }
        return element;
    }

    private Map<String, JsonElement> flattenJson(JsonObject object, String path) {
        Map<String, JsonElement> map = new LinkedHashMap<>();
        flattenInto(object, path, map);
        return map;
    }

    private void flattenInto(JsonObject object, String basePath, Map<String, JsonElement> out) {
        for (Map.Entry<String, JsonElement> entry : object.entrySet()) {
            String key = entry.getKey();
            JsonElement value = entry.getValue();
            String path = basePath.isEmpty() ? key : basePath + "." + key;
            if (value != null && value.isJsonObject()) {
                flattenInto(value.getAsJsonObject(), path, out);
            } else {
                out.put(path, value);
            }
        }
    }

    private JsonObject buildData(long timestampMs) {
        JsonObject data = new JsonObject();
        JsonObject meta = new JsonObject();
        meta.addProperty("sdk_version", sdkVersion);
        meta.addProperty("app_version", appVersion);
        meta.addProperty("device_model", deviceModel);
        meta.addProperty("active_opmode", getActiveOpModeName());
        meta.addProperty("opmode_state", getOpModeState().name());
        meta.addProperty("ts_ms", timestampMs);
        meta.addProperty("seq", sequence.incrementAndGet());
        data.add("meta", meta);

        JsonObject vitals = new JsonObject();
        double battery = getBatteryVoltage();
        if (!Double.isNaN(battery)) {
            vitals.addProperty("battery_v", round3(battery));
        }
        data.add("vitals", vitals);

        data.add("gamepad", buildGamepadSection());
        data.add("motors", buildMotorsSection());
        data.add("servos", buildServosSection());
        data.add("sensors", buildSensorsSection());
        data.add("telemetry", buildTelemetrySection());
        return data;
    }

    private JsonObject buildGamepadSection() {
        JsonObject root = new JsonObject();
        if (gamepad1 != null) {
            root.add("g1", serializeGamepad(gamepad1));
        }
        if (gamepad2 != null) {
            root.add("g2", serializeGamepad(gamepad2));
        }
        return root;
    }

    private JsonObject serializeGamepad(Gamepad gamepad) {
        JsonObject obj = new JsonObject();
        obj.addProperty("a", gamepad.a);
        obj.addProperty("b", gamepad.b);
        obj.addProperty("x", gamepad.x);
        obj.addProperty("y", gamepad.y);
        obj.addProperty("dpad_up", gamepad.dpad_up);
        obj.addProperty("dpad_down", gamepad.dpad_down);
        obj.addProperty("dpad_left", gamepad.dpad_left);
        obj.addProperty("dpad_right", gamepad.dpad_right);
        obj.addProperty("left_bumper", gamepad.left_bumper);
        obj.addProperty("right_bumper", gamepad.right_bumper);
        obj.addProperty("left_trigger", round3(gamepad.left_trigger));
        obj.addProperty("right_trigger", round3(gamepad.right_trigger));
        obj.addProperty("left_stick_button", gamepad.left_stick_button);
        obj.addProperty("right_stick_button", gamepad.right_stick_button);
        obj.addProperty("guide", gamepad.guide);
        obj.addProperty("start", gamepad.start);
        obj.addProperty("back", gamepad.back);
        obj.addProperty("left_stick_x", round3(gamepad.left_stick_x));
        obj.addProperty("left_stick_y", round3(gamepad.left_stick_y));
        obj.addProperty("right_stick_x", round3(gamepad.right_stick_x));
        obj.addProperty("right_stick_y", round3(gamepad.right_stick_y));
        obj.addProperty("timestamp", gamepad.timestamp);
        return obj;
    }

    private JsonObject buildMotorsSection() {
        JsonObject root = new JsonObject();
        for (Map.Entry<String, DcMotor> entry : motors.entrySet()) {
            String name = entry.getKey();
            DcMotor motor = entry.getValue();
            JsonObject obj = new JsonObject();
            try {
                obj.addProperty("power", round3(motor.getPower()));
            } catch (Exception ignored) { }
            try {
                obj.addProperty("position", motor.getCurrentPosition());
            } catch (Exception ignored) { }
            if (motor instanceof DcMotorEx) {
                DcMotorEx ex = (DcMotorEx) motor;
                try {
                    obj.addProperty("velocity", round3(ex.getVelocity()));
                } catch (Exception ignored) { }
                try {
                    double current = ex.getCurrent(CurrentUnit.AMPS);
                    if (Double.isFinite(current)) {
                        obj.addProperty("bus_current_a", round3(current));
                    }
                } catch (Exception ignored) { }
                try {
                    double temperature = Double.NaN;
                    try {
                        temperature = (double) DcMotorEx.class.getMethod("getMotorTemperature").invoke(ex);
                    } catch (Throwable ignored) {
                        // Method not present on this SDK/hub; leave as NaN or omit reporting
                    }

                    if (Double.isFinite(temperature)) {
                        obj.addProperty("temp_c", round3(temperature));
                    }
                } catch (Exception ignored) { }
            }
            root.add(name, obj);
        }
        return root;
    }

    private JsonObject buildServosSection() {
        JsonObject root = new JsonObject();
        for (Map.Entry<String, Servo> entry : servos.entrySet()) {
            JsonObject servoObj = new JsonObject();
            try {
                servoObj.addProperty("position", round3(entry.getValue().getPosition()));
            } catch (Exception ignored) { }
            root.add(entry.getKey(), servoObj);
        }
        for (Map.Entry<String, CRServo> entry : crServos.entrySet()) {
            JsonObject servoObj = root.getAsJsonObject(entry.getKey());
            if (servoObj == null) {
                servoObj = new JsonObject();
            }
            try {
                servoObj.addProperty("power", round3(entry.getValue().getPower()));
            } catch (Exception ignored) { }
            root.add(entry.getKey(), servoObj);
        }
        return root;
    }

    private JsonObject buildSensorsSection() {
        JsonObject root = new JsonObject();

        JsonObject imuObj = new JsonObject();
        for (Map.Entry<String, IMU> entry : imus.entrySet()) {
            JsonObject imuData = new JsonObject();
            try {
                YawPitchRollAngles ypr = entry.getValue().getRobotYawPitchRollAngles();
                imuData.addProperty("yaw", round3(ypr.getYaw(AngleUnit.DEGREES)));
                imuData.addProperty("pitch", round3(ypr.getPitch(AngleUnit.DEGREES)));
                imuData.addProperty("roll", round3(ypr.getRoll(AngleUnit.DEGREES)));
                imuData.addProperty("heading_deg", round3(ypr.getYaw(AngleUnit.DEGREES)));
            } catch (Exception ignored) { }
            try {
                AngularVelocity velocity = entry.getValue().getRobotAngularVelocity(AngleUnit.DEGREES);
                imuData.addProperty("angular_x_dps", round3(velocity.xRotationRate));
                imuData.addProperty("angular_y_dps", round3(velocity.yRotationRate));
                imuData.addProperty("angular_z_dps", round3(velocity.zRotationRate));
            } catch (Exception ignored) { }
            imuObj.add(entry.getKey(), imuData);
        }
        if (!imuObj.entrySet().isEmpty()) {
            root.add("imu", imuObj);
        }

        JsonObject distanceObj = new JsonObject();
        for (Map.Entry<String, DistanceSensor> entry : distanceSensors.entrySet()) {
            JsonObject distance = new JsonObject();
            try {
                double mm = entry.getValue().getDistance(DistanceUnit.MM);
                if (Double.isFinite(mm)) {
                    distance.addProperty("mm", round3(mm));
                }
            } catch (Exception ignored) { }
            distanceObj.add(entry.getKey(), distance);
        }
        if (!distanceObj.entrySet().isEmpty()) {
            root.add("distance", distanceObj);
        }

        JsonObject colorObj = new JsonObject();
        for (Map.Entry<String, com.qualcomm.robotcore.hardware.ColorSensor> entry : colorSensors.entrySet()) {
            com.qualcomm.robotcore.hardware.ColorSensor sensor = entry.getValue();
            JsonObject obj = new JsonObject();
            try { obj.addProperty("r", sensor.red()); } catch (Exception ignored) { }
            try { obj.addProperty("g", sensor.green()); } catch (Exception ignored) { }
            try { obj.addProperty("b", sensor.blue()); } catch (Exception ignored) { }
            try { obj.addProperty("alpha", sensor.alpha()); } catch (Exception ignored) { }
            colorObj.add(entry.getKey(), obj);
        }
        if (!colorObj.entrySet().isEmpty()) {
            root.add("color", colorObj);
        }

        JsonObject analogObj = new JsonObject();
        for (Map.Entry<String, AnalogInput> entry : analogInputs.entrySet()) {
            try {
                analogObj.addProperty(entry.getKey(), round3(entry.getValue().getVoltage()));
            } catch (Exception ignored) { }
        }
        if (!analogObj.entrySet().isEmpty()) {
            root.add("analog", analogObj);
        }

        JsonObject digitalObj = new JsonObject();
        for (Map.Entry<String, DigitalChannel> entry : digitalChannels.entrySet()) {
            try {
                digitalObj.addProperty(entry.getKey(), entry.getValue().getState());
            } catch (Exception ignored) { }
        }
        for (Map.Entry<String, TouchSensor> entry : touchSensors.entrySet()) {
            try {
                digitalObj.addProperty(entry.getKey(), entry.getValue().isPressed());
            } catch (Exception ignored) { }
        }
        if (!digitalObj.entrySet().isEmpty()) {
            root.add("digital", digitalObj);
        }

        JsonObject encoders = new JsonObject();
        for (Map.Entry<String, DcMotor> entry : motors.entrySet()) {
            try {
                encoders.addProperty(entry.getKey(), entry.getValue().getCurrentPosition());
            } catch (Exception ignored) { }
        }
        if (!encoders.entrySet().isEmpty()) {
            root.add("encoders", encoders);
        }

        return root;
    }

    private JsonObject buildTelemetrySection() {
        JsonObject telemetryObj = new JsonObject();
        JsonObject custom = new JsonObject();
        for (Map.Entry<String, String> entry : telemetryCustom.entrySet()) {
            custom.addProperty(entry.getKey(), entry.getValue());
        }
        telemetryObj.add("custom", custom);
        return telemetryObj;
    }

    private double getBatteryVoltage() {
        double best = Double.NaN;
        BatterySample override = batteryOverride.get();
        long now = System.currentTimeMillis();
        if (override.isFresh(now)) {
            best = override.voltage;
        }
        for (VoltageSensor sensor : voltageSensors) {
            try {
                double voltage = sensor.getVoltage();
                if (Double.isFinite(voltage)) {
                    if (Double.isNaN(best) || voltage > best) {
                        best = voltage;
                    }
                }
            } catch (Exception ignored) { }
        }
        if (Double.isNaN(best) && Double.isFinite(override.voltage)) {
            best = override.voltage;
        }
        return best;
    }

    private double round3(double value) {
        if (!Double.isFinite(value)) {
            return value;
        }
        return Math.round(value * 1000.0) / 1000.0;
    }

    private long getOpModeUptimeMs() {
        OpMode op = opMode;
        if (op == null) {
            return 0L;
        }
        try {
            double seconds = op.getRuntime();
            if (!Double.isFinite(seconds)) {
                return 0L;
            }
            long millis = (long) Math.round(seconds * 1000.0);
            return Math.max(0L, millis);
        } catch (Exception ignored) {
            return 0L;
        }
    }

    private String getActiveOpModeName() {
        OpMode op = opMode;
        if (op == null) return "NoOpMode";
        return op.getClass().getSimpleName();
    }

    public JsonObject getLatestSnapshotData() {
        synchronized (lock) {
            return latestSnapshotData == null ? new JsonObject() : GsonCompat.deepCopy(latestSnapshotData);
        }
    }

    public JsonObject getLatestSnapshotMessage() {
        synchronized (lock) {
            return latestSnapshotMessage == null ? null : GsonCompat.deepCopy(latestSnapshotMessage);
        }
    }

    private static final class BatterySample {
        static final BatterySample EMPTY = new BatterySample(Double.NaN, Long.MIN_VALUE);

        final double voltage;
        final long timestampMs;

        BatterySample(double voltage, long timestampMs) {
            this.voltage = voltage;
            this.timestampMs = timestampMs;
        }

        boolean isFresh(long nowMs) {
            return Double.isFinite(voltage) && (nowMs - timestampMs) <= BATTERY_OVERRIDE_STALE_MS;
        }
    }
}

