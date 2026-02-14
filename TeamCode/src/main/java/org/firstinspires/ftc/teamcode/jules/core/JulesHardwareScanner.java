package org.firstinspires.ftc.teamcode.jules.core;

import com.google.gson.JsonArray;
import com.google.gson.JsonObject;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.IMU;

import java.util.HashMap;
import java.util.Map;

/**
 * Dynamically scans the HardwareMap to discover connected devices.
 * Acts as a registry for "Universal OpMode" command execution and
 * generates the manifest for the JULES app.
 */
public class JulesHardwareScanner {

    // Dynamic Registries
    public final Map<String, DcMotor> motors = new HashMap<>();
    public final Map<String, Servo> servos = new HashMap<>();
    public final Map<String, CRServo> crServos = new HashMap<>();
    public final Map<String, DigitalChannel> digitalChannels = new HashMap<>();
    public final Map<String, VoltageSensor> voltageSensors = new HashMap<>();
    public final Map<String, IMU> imus = new HashMap<>();

    // Generic map for anything else if needed, or we can add specifics like sensors
    public final Map<String, HardwareDevice> allDevices = new HashMap<>();

    public void scan(HardwareMap map) {
        clear();

        // Scan ALL devices first to generic map
        for (HardwareMap.DeviceMapping<? extends HardwareDevice> mapping : map.allDeviceMappings) {
            for (Map.Entry<String, ? extends HardwareDevice> entry : mapping.entrySet()) {
                allDevices.put(entry.getKey(), entry.getValue());
            }
        }

        // Scan specific categories for typed access
        // Motors
        for (Map.Entry<String, DcMotor> entry : map.dcMotor.entrySet()) {
            motors.put(entry.getKey(), entry.getValue());
        }
        // Servos
        for (Map.Entry<String, Servo> entry : map.servo.entrySet()) {
            servos.put(entry.getKey(), entry.getValue());
        }
        // CR Servos
        for (Map.Entry<String, CRServo> entry : map.crservo.entrySet()) {
            crServos.put(entry.getKey(), entry.getValue());
        }
        // Digital Channels
        for (Map.Entry<String, DigitalChannel> entry : map.digitalChannel.entrySet()) {
            digitalChannels.put(entry.getKey(), entry.getValue());
        }
        // Voltage Sensors
        for (Map.Entry<String, VoltageSensor> entry : map.voltageSensor.entrySet()) {
            voltageSensors.put(entry.getKey(), entry.getValue());
        }
        // IMUs
        // IMUs
        for (Map.Entry<String, HardwareDevice> entry : allDevices.entrySet()) {
            if (entry.getValue() instanceof IMU) {
                imus.put(entry.getKey(), (IMU) entry.getValue());
            }
        }
    }

    public void clear() {
        motors.clear();
        servos.clear();
        crServos.clear();
        digitalChannels.clear();
        voltageSensors.clear();
        imus.clear();
        allDevices.clear();
    }

    public JsonObject getManifest() {
        JsonObject manifest = new JsonObject();

        manifest.add("motors", toJsonArray(motors.keySet()));
        manifest.add("servos", toJsonArray(servos.keySet()));
        manifest.add("cr_servos", toJsonArray(crServos.keySet()));
        manifest.add("digital_channels", toJsonArray(digitalChannels.keySet()));
        manifest.add("voltage_sensors", toJsonArray(voltageSensors.keySet()));
        manifest.add("imus", toJsonArray(imus.keySet()));

        // Add a summary of all devices for debugging/advanced usage
        JsonArray all = new JsonArray();
        for (String key : allDevices.keySet()) {
            all.add(key);
        }
        manifest.add("all_devices", all);

        return manifest;
    }

    private JsonArray toJsonArray(Iterable<String> keys) {
        JsonArray array = new JsonArray();
        for (String key : keys) {
            array.add(key);
        }
        return array;
    }

    // Command Execution Helpers

    public DcMotor getMotor(String id) {
        return motors.get(id);
    }

    public Servo getServo(String id) {
        return servos.get(id);
    }

    public CRServo getCRServo(String id) {
        return crServos.get(id);
    }
}
