package org.firstinspires.ftc.teamcode.jules.telemetry;

import android.content.Context;

import androidx.annotation.Nullable;
import android.os.Build;

import com.google.gson.JsonElement;
import com.google.gson.JsonNull;
import com.google.gson.JsonObject;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.AtomicLong;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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

/**
 * Centralized aggregator that collects hardware state, meta data, and mirrored telemetry entries.
 *
 * <p>The organizer caches hardware devices during {@link #bind(HardwareMap, Gamepad, Gamepad, OpMode)}
 * and builds JSON payloads on demand. All JSON objects returned are immutable snapshots.</p>
 */
public final class JulesDataOrganizer {

    private static final JulesDataOrganizer INSTANCE = new JulesDataOrganizer();

    private final AtomicLong sequence = new AtomicLong();
    private final Map<String, String> customTelemetry = new ConcurrentHashMap<>();
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.BuildConfig;

import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.Map;
import java.util.Objects;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.AtomicLong;

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

    private final AtomicLong sequence = new AtomicLong();

    private HardwareMap hardwareMap;
    private Gamepad gamepad1;
    private Gamepad gamepad2;
    private OpMode opMode;

    private String activeOpMode = "";
    private String opModeState = "STOPPED";

    private VoltageSensor primaryVoltageSensor;
    private List<DcMotorEx> motors = Collections.emptyList();
    private List<Servo> servos = Collections.emptyList();
    private List<CRServo> crServos = Collections.emptyList();
    private List<DistanceSensor> distanceSensors = Collections.emptyList();
    private List<ColorSensor> colorSensors = Collections.emptyList();
    private List<TouchSensor> touchSensors = Collections.emptyList();
    private List<AnalogInput> analogInputs = Collections.emptyList();
    private List<com.qualcomm.robotcore.hardware.DigitalChannel> digitalChannels = Collections.emptyList();
    private IMU imu;

    private JsonObject lastDataSnapshot = null;
    private JulesTelemetry telemetryMirror;

    private JulesDataOrganizer() { }

    public static JulesDataOrganizer getInstance() {
        return INSTANCE;
    }

    public synchronized void bind(HardwareMap map, Gamepad g1, Gamepad g2, OpMode mode) {
        this.hardwareMap = map;
        this.gamepad1 = g1;
        this.gamepad2 = g2;
        this.opMode = mode;
        this.activeOpMode = mode != null ? mode.getClass().getSimpleName() : "";
        if (map != null) {
            this.motors = new ArrayList<>(map.getAll(DcMotorEx.class));
            this.servos = new ArrayList<>(map.getAll(Servo.class));
            this.crServos = new ArrayList<>(map.getAll(CRServo.class));
            this.distanceSensors = new ArrayList<>(map.getAll(DistanceSensor.class));
            this.colorSensors = new ArrayList<>(map.getAll(ColorSensor.class));
            this.touchSensors = new ArrayList<>(map.getAll(TouchSensor.class));
            this.analogInputs = new ArrayList<>(map.getAll(AnalogInput.class));
            this.digitalChannels = new ArrayList<>(map.getAll(com.qualcomm.robotcore.hardware.DigitalChannel.class));

            this.primaryVoltageSensor = null;
            for (VoltageSensor sensor : map.voltageSensor) {
                this.primaryVoltageSensor = sensor;
                break;
            }

            List<IMU> imuList = new ArrayList<>(map.getAll(IMU.class));
            if (!imuList.isEmpty()) {
                this.imu = imuList.get(0);
            }
        }
    }

    public synchronized void setOpModeState(String state) {
        this.opModeState = state;
    }

    public synchronized void updateGamepads(Gamepad g1, Gamepad g2) {
        this.gamepad1 = g1;
        this.gamepad2 = g2;
    }

    public synchronized void mirrorTelemetry(Telemetry telemetry) {
        if (telemetry == null) {
            this.telemetryMirror = null;
            return;
        }
        this.telemetryMirror = new JulesTelemetry(telemetry, this);
    }

    @Nullable
    public synchronized Telemetry getMirroredTelemetry() {
        return telemetryMirror != null ? telemetryMirror.getFtcTelemetry() : null;
    }

    public void recordTelemetry(String key, String value) {
        if (key == null) {
            return;
        }
        if (value == null) {
            customTelemetry.remove(key);
        } else {
            customTelemetry.put(key, value);
        }
    }

    public void clearTelemetry() {
        customTelemetry.clear();
    }

    public synchronized JsonObject buildSnapshot() {
        long ts = System.currentTimeMillis();
        JsonObject payload = new JsonObject();
        payload.addProperty("type", "snapshot");
        payload.addProperty("ts_ms", ts);
        payload.addProperty("active_opmode", activeOpMode);

        JsonObject data = buildData(ts);
        payload.add("data", data);
        lastDataSnapshot = data.deepCopy();
        return payload;
    }

    public synchronized JsonObject buildHeartbeat() {
        long ts = System.currentTimeMillis();
        JsonObject heartbeat = new JsonObject();
        heartbeat.addProperty("type", "heartbeat");
        heartbeat.addProperty("ts_ms", ts);
        heartbeat.addProperty("seq", sequence.incrementAndGet());
        heartbeat.addProperty("battery_v", readBatteryVoltage());
        heartbeat.addProperty("active_opmode", activeOpMode);
        return heartbeat;
    }

    @Nullable
    public synchronized JsonObject buildDiffSince(@Nullable JsonObject previousData) {
        long ts = System.currentTimeMillis();
        JsonObject currentData = buildData(ts);
        JsonObject patch = new JsonObject();
        diffObjects("", previousData, currentData, patch);
        if (patch.size() == 0) {
            lastDataSnapshot = currentData.deepCopy();
            return null;
        }

        JsonObject diff = new JsonObject();
        diff.addProperty("type", "diff");
        diff.addProperty("ts_ms", ts);
        diff.add("patch", patch);
        lastDataSnapshot = currentData.deepCopy();
        return diff;
    }

    @Nullable
    public synchronized JsonObject getLastDataSnapshot() {
        return lastDataSnapshot == null ? null : lastDataSnapshot.deepCopy();
    }

    private JsonObject buildData(long ts) {
        JsonObject data = new JsonObject();
        data.add("meta", buildMeta(ts));
        data.add("vitals", buildVitals());
        data.add("gamepad", buildGamepads());
        data.add("motors", buildMotors());
        data.add("servos", buildServos());
        data.add("sensors", buildSensors());
        data.add("telemetry", buildTelemetry());
        return data;
    }

    private JsonObject buildMeta(long ts) {
        JsonObject meta = new JsonObject();
        meta.addProperty("sdk_version", getSdkVersion());
        meta.addProperty("app_version", getAppVersion());
        meta.addProperty("active_opmode", activeOpMode);
        meta.addProperty("opmode_state", opModeState);
        meta.addProperty("ts_ms", ts);
        meta.addProperty("seq", sequence.incrementAndGet());
        return meta;
    }

    private JsonObject buildVitals() {
        JsonObject vitals = new JsonObject();
        double voltage = readBatteryVoltage();
        if (!Double.isNaN(voltage)) {
            vitals.addProperty("battery_v", voltage);
        }

        JsonObject temps = new JsonObject();
        if (hardwareMap != null) {
            for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
                try {
                    String name = getDeviceName(module);
                    double temp = module.getCurrentTemperature();
                    if (!Double.isNaN(temp)) {
                        temps.addProperty(name, temp);
                    }
                } catch (Throwable t) {
                    RobotLog.ee("JulesDataOrganizer", t, "Unable to read hub temperature");
                }
            }
        }
        if (temps.size() > 0) {
            vitals.add("hub_temps_c", temps);
        }
        return vitals;
    }

    private JsonObject buildGamepads() {
        JsonObject gamepads = new JsonObject();
        gamepads.add("g1", serializeGamepad(gamepad1));
        gamepads.add("g2", serializeGamepad(gamepad2));
        return gamepads;
    }

    private JsonObject serializeGamepad(@Nullable Gamepad gamepad) {
        JsonObject obj = new JsonObject();
        if (gamepad == null) {
            return obj;
        }
    private OpModeState opModeState = OpModeState.INIT;
    private String sdkVersion = "UNKNOWN";
    private String appVersion = "UNKNOWN";
    private String deviceModel = Build.MODEL == null ? "UNKNOWN" : Build.MODEL;

    private JsonObject latestSnapshotData = new JsonObject();
    private JsonObject latestSnapshotMessage = null;

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
        try {
            Context context = map.appContext;
            if (context != null) {
                sdkVersion = RobotLog.getVersion();
            }
        } catch (Exception e) {
            sdkVersion = "UNKNOWN";
        }
        try {
            appVersion = BuildConfig.VERSION_NAME;
        } catch (Exception ignored) {
            appVersion = "UNKNOWN";
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
        heartbeat.addProperty("battery_v", getBatteryVoltage());
        heartbeat.addProperty("active_opmode", getActiveOpModeName());
        return heartbeat;
    }

    public JsonObject buildSnapshot() {
        JsonObject data;
        long now = System.currentTimeMillis();
        synchronized (lock) {
            data = buildData(now);
            latestSnapshotData = data.deepCopy();
        }

        JsonObject snapshot = new JsonObject();
        snapshot.addProperty("type", "snapshot");
        snapshot.addProperty("ts_ms", now);
        snapshot.addProperty("active_opmode", getActiveOpModeName());
        snapshot.add("data", data);
        synchronized (lock) {
            latestSnapshotMessage = snapshot.deepCopy();
        }
        return snapshot;
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
            latestSnapshotData = newData.deepCopy();
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
        obj.addProperty("back", gamepad.back);
        obj.addProperty("start", gamepad.start);
        obj.addProperty("guide", gamepad.guide);
        obj.addProperty("dpad_up", gamepad.dpad_up);
        obj.addProperty("dpad_down", gamepad.dpad_down);
        obj.addProperty("dpad_left", gamepad.dpad_left);
        obj.addProperty("dpad_right", gamepad.dpad_right);
        obj.addProperty("left_bumper", gamepad.left_bumper);
        obj.addProperty("right_bumper", gamepad.right_bumper);
        obj.addProperty("left_trigger", gamepad.left_trigger);
        obj.addProperty("right_trigger", gamepad.right_trigger);
        obj.addProperty("left_stick_x", (double) gamepad.left_stick_x);
        obj.addProperty("left_stick_y", (double) gamepad.left_stick_y);
        obj.addProperty("right_stick_x", (double) gamepad.right_stick_x);
        obj.addProperty("right_stick_y", (double) gamepad.right_stick_y);
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

    private JsonObject buildMotors() {
        JsonObject motorsObj = new JsonObject();
        if (motors == null) {
            return motorsObj;
        }
        for (DcMotorEx motor : motors) {
            String name = getDeviceName(motor);
            if (name == null) {
                continue;
            }
            JsonObject m = new JsonObject();
            try {
                m.addProperty("power", motor.getPower());
            } catch (Throwable ignored) {
            }
            try {
                m.addProperty("velocity", motor.getVelocity());
            } catch (Throwable ignored) {
            }
            try {
                m.addProperty("position", motor.getCurrentPosition());
            } catch (Throwable ignored) {
            }
            try {
                double current = motor.getCurrent(com.qualcomm.robotcore.hardware.CurrentUnit.AMPS);
                if (!Double.isNaN(current)) {
                    m.addProperty("bus_current_a", current);
                }
            } catch (Throwable ignored) {
            }
            try {
                double temp = motor.getMotorTemperature(com.qualcomm.robotcore.hardware.TemperatureUnit.CELSIUS);
                if (!Double.isNaN(temp)) {
                    m.addProperty("temp_c", temp);
                }
            } catch (Throwable ignored) {
            }
            motorsObj.add(name, m);
        }
        return motorsObj;
    }

    private JsonObject buildServos() {
        JsonObject servosObj = new JsonObject();
        if (servos != null) {
            for (Servo servo : servos) {
                String name = getDeviceName(servo);
                if (name == null) {
                    continue;
                }
                JsonObject s = new JsonObject();
                try {
                    s.addProperty("position", servo.getPosition());
                } catch (Throwable ignored) {
                }
                servosObj.add(name, s);
            }
        }
        if (crServos != null) {
            for (CRServo servo : crServos) {
                String name = getDeviceName(servo);
                if (name == null) {
                    continue;
                }
                JsonObject s = new JsonObject();
                try {
                    s.addProperty("power", servo.getPower());
                } catch (Throwable ignored) {
                }
                servosObj.add(name, s);
            }
        }
        return servosObj;
    }

    private JsonObject buildSensors() {
        JsonObject sensors = new JsonObject();
        JsonObject imuObj = buildImu();
        if (imuObj.size() > 0) {
            sensors.add("imu", imuObj);
        }
        JsonObject distanceObj = new JsonObject();
        if (distanceSensors != null) {
            for (DistanceSensor sensor : distanceSensors) {
                String name = getDeviceName(sensor);
                if (name == null) {
                    continue;
                }
                JsonObject entry = new JsonObject();
                try {
                    double mm = sensor.getDistance(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.MM);
                    if (!Double.isNaN(mm)) {
                        entry.addProperty("mm", mm);
                    }
                } catch (Throwable ignored) {
                }
                if (entry.size() > 0) {
                    distanceObj.add(name, entry);
                }
            }
        }
        if (distanceObj.size() > 0) {
            sensors.add("distance", distanceObj);
        }

        JsonObject colorObj = new JsonObject();
        if (colorSensors != null) {
            for (ColorSensor sensor : colorSensors) {
                String name = getDeviceName(sensor);
                if (name == null) {
                    continue;
                }
                JsonObject entry = new JsonObject();
                try {
                    entry.addProperty("r", sensor.red());
                    entry.addProperty("g", sensor.green());
                    entry.addProperty("b", sensor.blue());
                    entry.addProperty("alpha", sensor.alpha());
                } catch (Throwable ignored) {
                }
                colorObj.add(name, entry);
            }
        }
        if (colorObj.size() > 0) {
            sensors.add("color", colorObj);
        }

        JsonObject analogObj = new JsonObject();
        if (analogInputs != null) {
            for (AnalogInput input : analogInputs) {
                String name = getDeviceName(input);
                if (name == null) {
                    continue;
                }
                try {
                    analogObj.addProperty(name, input.getVoltage());
                } catch (Throwable ignored) {
                }
            }
        }
        if (analogObj.size() > 0) {
            sensors.add("analog", analogObj);
        }

        JsonObject digitalObj = new JsonObject();
        if (digitalChannels != null) {
            for (com.qualcomm.robotcore.hardware.DigitalChannel channel : digitalChannels) {
                String name = getDeviceName(channel);
                if (name == null) {
                    continue;
                }
                try {
                    digitalObj.addProperty(name, channel.getState());
                } catch (Throwable ignored) {
                }
            }
        }
        if (digitalObj.size() > 0) {
            sensors.add("digital", digitalObj);
        }

        return sensors;
    }

    private JsonObject buildTelemetry() {
        JsonObject telemetry = new JsonObject();
        JsonObject custom = new JsonObject();
        for (Map.Entry<String, String> entry : customTelemetry.entrySet()) {
            custom.addProperty(entry.getKey(), entry.getValue());
        }
        telemetry.add("custom", custom);
        return telemetry;
    }

    private JsonObject buildImu() {
        JsonObject obj = new JsonObject();
        if (imu == null) {
            return obj;
        }
        try {
            YawPitchRollAngles ypr = imu.getRobotYawPitchRollAngles();
            obj.addProperty("yaw", ypr.getYaw(AngleUnit.DEGREES));
            obj.addProperty("pitch", ypr.getPitch(AngleUnit.DEGREES));
            obj.addProperty("roll", ypr.getRoll(AngleUnit.DEGREES));
            obj.addProperty("heading_deg", ypr.getYaw(AngleUnit.DEGREES));
        } catch (Throwable ignored) {
        }
        try {
            AngularVelocity velocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
            obj.addProperty("vel_x", velocity.xRotationRate);
            obj.addProperty("vel_y", velocity.yRotationRate);
            obj.addProperty("vel_z", velocity.zRotationRate);
        } catch (Throwable ignored) {
        }
        return obj;
    }

    private double readBatteryVoltage() {
        try {
            if (primaryVoltageSensor != null) {
                return primaryVoltageSensor.getVoltage();
            }
        } catch (Throwable ignored) {
        }
        return Double.NaN;
    }

    private void diffObjects(String prefix, @Nullable JsonObject previous, JsonObject current, JsonObject patch) {
        if (current == null) {
            return;
        }
        Set<String> keys = new HashSet<>();
        if (current != null) {
            keys.addAll(current.keySet());
        }
        if (previous != null) {
            keys.addAll(previous.keySet());
        }

        for (String key : keys) {
            JsonElement currValue = current != null && current.has(key) ? current.get(key) : null;
            JsonElement prevValue = previous != null && previous.has(key) ? previous.get(key) : null;

            String childPath = prefix == null || prefix.isEmpty() ? key : prefix + "." + key;

            if (currValue == null) {
                patch.add(childPath, JsonNull.INSTANCE);
                continue;
            }

            if (prevValue == null) {
                patch.add(childPath, currValue.deepCopy());
                continue;
            }

            if (currValue.isJsonObject() && prevValue.isJsonObject()) {
                diffObjects(childPath, prevValue.getAsJsonObject(), currValue.getAsJsonObject(), patch);
            } else if (!Objects.equals(currValue, prevValue)) {
                patch.add(childPath, currValue.deepCopy());
            }
        }
    }

    private String getSdkVersion() {
        try {
            return com.qualcomm.robotcore.BuildConfig.VERSION_NAME;
        } catch (Throwable ignored) {
        }
        return "unknown";
    }

    private String getAppVersion() {
        if (opMode == null) {
            return "unknown";
        }
        try {
            Context context = opMode.hardwareMap.appContext;
            if (context == null) {
                return "unknown";
            }
            return context.getPackageManager()
                    .getPackageInfo(context.getPackageName(), 0)
                    .versionName;
        } catch (Throwable ignored) {
            return "unknown";
        }
    }

    private String getDeviceName(HardwareDevice device) {
        if (hardwareMap == null || device == null) {
            return null;
        }
        try {
            Set<String> names = hardwareMap.getNamesOf(device);
            if (!names.isEmpty()) {
                return names.iterator().next();
            }
        } catch (Throwable ignored) {
        }
        try {
            return device.getDeviceName();
        } catch (Throwable ignored) {
        }
        return null;
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
                    double current = ex.getCurrent(DcMotorEx.CurrentUnit.AMPS);
                    if (Double.isFinite(current)) {
                        obj.addProperty("bus_current_a", round3(current));
                    }
                } catch (Exception ignored) { }
                try {
                    double temperature = ex.getMotorTemperature();
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
        return best;
    }

    private double round3(double value) {
        if (!Double.isFinite(value)) {
            return value;
        }
        return Math.round(value * 1000.0) / 1000.0;
    }

    private String getActiveOpModeName() {
        OpMode op = opMode;
        if (op == null) return "NoOpMode";
        return op.getClass().getSimpleName();
    }

    public JsonObject getLatestSnapshotData() {
        synchronized (lock) {
            return latestSnapshotData == null ? new JsonObject() : latestSnapshotData.deepCopy();
        }
    }

    public JsonObject getLatestSnapshotMessage() {
        synchronized (lock) {
            return latestSnapshotMessage == null ? null : latestSnapshotMessage.deepCopy();
        }
    }
}

