package org.firstinspires.ftc.teamcode.jules.telemetry;

import android.content.Context;

import androidx.annotation.Nullable;

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
    }
}

