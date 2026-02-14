package org.firstinspires.ftc.teamcode.jules.shot;

import androidx.annotation.Nullable;

import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.jules.bridge.util.GsonCompat;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

/**
 * Computes flywheel RPM targets from distance and battery voltage.
 */
public final class RpmProvider {

    public static final class Target {
        /** Base model output after voltage scaling (no delta bins or session bias). */
        public final double rpmBase;
        /** Nominal target before planner bias (base + delta bins + session bias). */
        public final double rpmTarget;

        Target(double rpmBase, double rpmTarget) {
            this.rpmBase = rpmBase;
            this.rpmTarget = rpmTarget;
        }
    }

    private static final double MIN_VOLTAGE = 10.5;
    private static final double MAX_VOLTAGE = 13.0;
    private static final long LOAD_WINDOW_MS = 300L;
    private static final String TAG = "RpmProvider";

    private interface BaseModel {
        double eval(double distanceIn);

        String describe();
    }

    private static final class QuadraticModel implements BaseModel {
        private final double b0;
        private final double b1;
        private final double b2;

        QuadraticModel(double b0, double b1, double b2) {
            this.b0 = b0;
            this.b1 = b1;
            this.b2 = b2;
        }

        @Override
        public double eval(double distanceIn) {
            return b0 + (b1 * distanceIn) + (b2 * distanceIn * distanceIn);
        }

        @Override
        public String describe() {
            return String.format(Locale.US, "quadratic(b0=%.3f,b1=%.3f,b2=%.5f)", b0, b1, b2);
        }
    }

    private static final class PowerModel implements BaseModel {
        private final double alpha;
        private final double beta;
        private final double p;

        PowerModel(double alpha, double beta, double p) {
            this.alpha = alpha;
            this.beta = beta;
            this.p = p;
        }

        @Override
        public double eval(double distanceIn) {
            double shifted = Math.max(0.0, distanceIn + beta);
            return alpha * Math.pow(shifted, p);
        }

        @Override
        public String describe() {
            return String.format(Locale.US, "power(alpha=%.3f,beta=%.3f,p=%.3f)", alpha, beta, p);
        }
    }

    private static final class LinearFeetModel implements BaseModel {
        private final double slopePerFt;
        private final double offset;

        LinearFeetModel(double slopePerFt, double offset) {
            this.slopePerFt = slopePerFt;
            this.offset = offset;
        }

        @Override
        public double eval(double distanceIn) {
            return offset + slopePerFt * (distanceIn / 12.0);
        }

        @Override
        public String describe() {
            return String.format(Locale.US, "legacy_linear(m=%.3f,b=%.3f)", slopePerFt, offset);
        }
    }

    private static final class DeltaBin {
        final double minDist;
        final double maxDist;
        final double minV;
        final double maxV;
        final double delta;
        final double centerDist;
        final double centerV;

        DeltaBin(double minDist, double maxDist, double minV, double maxV, double delta) {
            this.minDist = Math.min(minDist, maxDist);
            this.maxDist = Math.max(minDist, maxDist);
            this.minV = Math.min(minV, maxV);
            this.maxV = Math.max(minV, maxV);
            this.delta = delta;
            this.centerDist = (this.minDist + this.maxDist) * 0.5;
            this.centerV = (this.minV + this.maxV) * 0.5;
        }

        boolean contains(double distance, double voltage) {
            return distance >= minDist && distance <= maxDist
                    && voltage >= minV && voltage <= maxV;
        }
    }

    private static final class VoltageSample {
        final long timestamp;
        final double voltage;

        VoltageSample(long timestamp, double voltage) {
            this.timestamp = timestamp;
            this.voltage = voltage;
        }
    }

    private final ArrayDeque<VoltageSample> loadSamples = new ArrayDeque<>();
    private final Map<String, DeltaBin> binGrid = new HashMap<>();

    private BaseModel baseModel = new LinearFeetModel(110.0, 2000.0);
    private double gamma = 1.0;
    private double vNom = 12.0;
    private double sessionBias = 0.0;
    private int modelVersion = 0;

    private List<DeltaBin> deltaBins = Collections.emptyList();
    private double[] deltaDistCenters = new double[0];
    private double[] deltaVoltCenters = new double[0];

    private double lastBase = 0.0;
    private double lastTarget = 0.0;

    public synchronized double getLastBaseRpm() {
        return lastBase;
    }

    public synchronized double getLastTargetRpm() {
        return lastTarget;
    }

    public synchronized double getSessionBias() {
        return sessionBias;
    }

    public synchronized void setSessionBias(double bias) {
        sessionBias = bias;
    }

    public synchronized void clearSessionBias() {
        sessionBias = 0.0;
    }

    public synchronized int getModelVersion() {
        return modelVersion;
    }

    /** Update the rolling load-voltage median and return the filtered value. */
    public synchronized double updateAndGetLoadVoltage(long nowMs, double measured, boolean underLoad) {
        double clamped = clamp(measured, MIN_VOLTAGE, MAX_VOLTAGE);
        purgeOldVoltageSamples(nowMs);
        if (underLoad) {
            loadSamples.addLast(new VoltageSample(nowMs, clamped));
        }
        if (loadSamples.isEmpty()) {
            return clamped;
        }
        double[] values = new double[loadSamples.size()];
        int i = 0;
        for (VoltageSample sample : loadSamples) {
            values[i++] = sample.voltage;
        }
        Arrays.sort(values);
        int mid = values.length / 2;
        if ((values.length & 1) == 0) {
            return (values[mid - 1] + values[mid]) * 0.5;
        }
        return values[mid];
    }

    private void purgeOldVoltageSamples(long nowMs) {
        while (!loadSamples.isEmpty()) {
            VoltageSample sample = loadSamples.peekFirst();
            if (sample == null || nowMs - sample.timestamp <= LOAD_WINDOW_MS) {
                break;
            }
            loadSamples.removeFirst();
        }
    }

    /** Compute the RPM target for the supplied range and filtered battery voltage. */
    public synchronized Target target(double distanceIn, double vBattLoad) {
        double distance = Math.max(0.0, distanceIn);
        double voltage = clamp(vBattLoad, MIN_VOLTAGE, MAX_VOLTAGE);

        double base = baseModel.eval(distance);
        double scaled = base;
        if (voltage > 1e-6) {
            scaled = base * Math.pow(vNom / voltage, gamma);
        }

        double delta = computeDelta(distance, voltage);
        double rpmBase = Double.isFinite(scaled) ? scaled : base;
        double rpmTarget = rpmBase + delta + sessionBias;
        if (!Double.isFinite(rpmTarget)) {
            rpmTarget = rpmBase;
        }

        lastBase = rpmBase;
        lastTarget = rpmTarget;
        return new Target(rpmBase, rpmTarget);
    }

    private double computeDelta(double distanceIn, double voltage) {
        if (deltaBins.isEmpty()) {
            return 0.0;
        }
        DeltaBin best = null;
        double bestScore = Double.MAX_VALUE;
        for (DeltaBin bin : deltaBins) {
            if (!bin.contains(distanceIn, voltage)) {
                continue;
            }
            double score = Math.abs(bin.centerDist - distanceIn) + Math.abs(bin.centerV - voltage);
            if (score < bestScore) {
                bestScore = score;
                best = bin;
            }
        }
        double baseDelta = (best != null) ? best.delta : 0.0;

        // Attempt bilinear interpolation using surrounding bin centers.
        if (deltaDistCenters.length >= 2 && deltaVoltCenters.length >= 2) {
            int di0 = floorIndex(deltaDistCenters, distanceIn);
            int di1 = ceilIndex(deltaDistCenters, distanceIn);
            int vi0 = floorIndex(deltaVoltCenters, voltage);
            int vi1 = ceilIndex(deltaVoltCenters, voltage);
            if (di0 >= 0 && di1 >= 0 && vi0 >= 0 && vi1 >= 0) {
                String k00 = key(deltaDistCenters[di0], deltaVoltCenters[vi0]);
                String k10 = key(deltaDistCenters[di1], deltaVoltCenters[vi0]);
                String k01 = key(deltaDistCenters[di0], deltaVoltCenters[vi1]);
                String k11 = key(deltaDistCenters[di1], deltaVoltCenters[vi1]);
                DeltaBin b00 = binGrid.get(k00);
                DeltaBin b10 = binGrid.get(k10);
                DeltaBin b01 = binGrid.get(k01);
                DeltaBin b11 = binGrid.get(k11);
                if (b00 != null && b10 != null && b01 != null && b11 != null
                        && b00.contains(distanceIn, voltage)
                        && b10.contains(distanceIn, voltage)
                        && b01.contains(distanceIn, voltage)
                        && b11.contains(distanceIn, voltage)) {
                    double d0 = deltaDistCenters[di0];
                    double d1 = deltaDistCenters[di1];
                    double v0 = deltaVoltCenters[vi0];
                    double v1 = deltaVoltCenters[vi1];
                    double t = (d1 - d0) == 0 ? 0.0 : (distanceIn - d0) / (d1 - d0);
                    double u = (v1 - v0) == 0 ? 0.0 : (voltage - v0) / (v1 - v0);
                    t = clamp01(t);
                    u = clamp01(u);
                    double interp = (1 - t) * (1 - u) * b00.delta
                            + t * (1 - u) * b10.delta
                            + (1 - t) * u * b01.delta
                            + t * u * b11.delta;
                    return interp;
                }
            }
        }
        return baseDelta;
    }

    private static int floorIndex(double[] arr, double value) {
        if (arr.length == 0 || value < arr[0]) {
            return 0;
        }
        for (int i = arr.length - 1; i >= 0; i--) {
            if (value >= arr[i]) {
                return i;
            }
        }
        return arr.length - 1;
    }

    private static int ceilIndex(double[] arr, double value) {
        if (arr.length == 0) {
            return -1;
        }
        if (value > arr[arr.length - 1]) {
            return arr.length - 1;
        }
        for (int i = 0; i < arr.length; i++) {
            if (value <= arr[i]) {
                return i;
            }
        }
        return arr.length - 1;
    }

    private static double clamp01(double v) {
        if (v < 0.0) return 0.0;
        if (v > 1.0) return 1.0;
        return v;
    }

    /** Apply a JSON model update using the rpm_model/v1 schema. */
    public synchronized void applyUpdate(JsonObject update) {
        if (update == null) {
            return;
        }

        String schema = optString(update, "schema");
        if (schema != null && !"rpm_model/v1".equalsIgnoreCase(schema)) {
            RobotLog.w(TAG, "Ignoring rpm model with unsupported schema %s", schema);
            return;
        }

        int incomingVersion = optInt(update, "model_version", modelVersion);
        if (incomingVersion <= modelVersion) {
            RobotLog.v(TAG, "Ignoring rpm model version %d (current=%d)", incomingVersion, modelVersion);
            return;
        }

        BaseModel newBase = parseBase(update, baseModel);
        if (newBase != null) {
            baseModel = newBase;
        }

        gamma = optDouble(update, "gamma", gamma);
        vNom = optDouble(update, "v_nom", vNom);

        List<DeltaBin> newBins = parseDeltaBins(update);
        if (newBins != null) {
            deltaBins = newBins;
            rebuildDeltaGrid();
        }

        modelVersion = incomingVersion;
        RobotLog.ii(TAG, "Loaded rpm_model/v1 version %d using %s", modelVersion, baseModel.describe());
    }

    private void rebuildDeltaGrid() {
        binGrid.clear();
        if (deltaBins.isEmpty()) {
            deltaDistCenters = new double[0];
            deltaVoltCenters = new double[0];
            return;
        }
        List<Double> distCenters = new ArrayList<>();
        List<Double> voltCenters = new ArrayList<>();
        for (DeltaBin bin : deltaBins) {
            String key = key(bin.centerDist, bin.centerV);
            binGrid.put(key, bin);
            if (!contains(distCenters, bin.centerDist)) {
                distCenters.add(bin.centerDist);
            }
            if (!contains(voltCenters, bin.centerV)) {
                voltCenters.add(bin.centerV);
            }
        }
        Collections.sort(distCenters);
        Collections.sort(voltCenters);
        deltaDistCenters = toArray(distCenters);
        deltaVoltCenters = toArray(voltCenters);
    }

    private static boolean contains(List<Double> list, double value) {
        for (Double existing : list) {
            if (existing != null && Math.abs(existing - value) < 1e-6) {
                return true;
            }
        }
        return false;
    }

    private static double[] toArray(List<Double> list) {
        double[] arr = new double[list.size()];
        for (int i = 0; i < list.size(); i++) {
            arr[i] = list.get(i);
        }
        return arr;
    }

    private static String key(double dist, double volt) {
        return String.format(Locale.US, "%.4f/%.4f", dist, volt);
    }

    private static BaseModel parseBase(JsonObject update, BaseModel fallback) {
        JsonObject baseObj = asObject(update.get("base"));
        if (baseObj != null) {
            String family = optString(baseObj, "family");
            if (family != null) {
                if ("quadratic".equalsIgnoreCase(family)) {
                    double b0 = optDouble(baseObj, "b0", 0.0);
                    double b1 = optDouble(baseObj, "b1", 0.0);
                    double b2 = optDouble(baseObj, "b2", 0.0);
                    return new QuadraticModel(b0, b1, b2);
                }
                if ("power".equalsIgnoreCase(family)) {
                    double alpha = optDouble(baseObj, "alpha", 0.0);
                    double beta = optDouble(baseObj, "beta", 0.0);
                    double p = optDouble(baseObj, "p", 1.0);
                    return new PowerModel(alpha, beta, p);
                }
            }
        }

        if (update.has("m_rpm_per_ft") || update.has("b_rpm_offset")) {
            double m = optDouble(update, "m_rpm_per_ft", 110.0);
            double b = optDouble(update, "b_rpm_offset", 2000.0);
            return new LinearFeetModel(m, b);
        }
        return fallback;
    }

    @Nullable
    private static List<DeltaBin> parseDeltaBins(JsonObject update) {
        if (update == null || !update.has("delta_bins")) {
            return null;
        }
        JsonArray arr = safeArray(update.get("delta_bins"));
        if (arr == null) {
            return null;
        }
        List<DeltaBin> bins = new ArrayList<>();
        for (JsonElement element : arr) {
            JsonObject binObj = asObject(element);
            if (binObj == null) {
                continue;
            }
            double[] dRange = readRange(binObj.get("d"));
            double[] vRange = readRange(binObj.get("v"));
            double delta = optDouble(binObj, "delta", 0.0);
            if (dRange != null && vRange != null) {
                bins.add(new DeltaBin(dRange[0], dRange[1], vRange[0], vRange[1], delta));
            }
        }
        return bins;
    }

    private static JsonArray safeArray(JsonElement el) {
        if (el == null) {
            return null;
        }
        if (el.isJsonArray()) {
            return el.getAsJsonArray();
        }
        if (el.isJsonPrimitive()) {
            JsonElement parsed = GsonCompat.parse(el.getAsString());
            if (parsed != null && parsed.isJsonArray()) {
                return parsed.getAsJsonArray();
            }
        }
        return null;
    }

    private static JsonObject asObject(JsonElement el) {
        if (el == null) {
            return null;
        }
        if (el.isJsonObject()) {
            return el.getAsJsonObject();
        }
        if (el.isJsonPrimitive()) {
            JsonElement parsed = GsonCompat.parse(el.getAsString());
            if (parsed != null && parsed.isJsonObject()) {
                return parsed.getAsJsonObject();
            }
        }
        return null;
    }

    private static double[] readRange(JsonElement el) {
        JsonArray arr = safeArray(el);
        if (arr == null || arr.size() < 2) {
            return null;
        }
        double lo = optDouble(arr.get(0), Double.NaN);
        double hi = optDouble(arr.get(1), Double.NaN);
        if (Double.isNaN(lo) || Double.isNaN(hi)) {
            return null;
        }
        return new double[]{lo, hi};
    }

    private static double optDouble(JsonObject obj, String field, double def) {
        if (obj == null || field == null) {
            return def;
        }
        return optDouble(obj.get(field), def);
    }

    private static double optDouble(JsonElement el, double def) {
        if (el == null) {
            return def;
        }
        try {
            return el.getAsDouble();
        } catch (Exception ignored) {
            if (el.isJsonPrimitive()) {
                try {
                    return Double.parseDouble(el.getAsString());
                } catch (Exception ignored2) {
                    return def;
                }
            }
            return def;
        }
    }

    private static int optInt(JsonObject obj, String field, int def) {
        if (obj == null || field == null) {
            return def;
        }
        JsonElement el = obj.get(field);
        if (el == null) {
            return def;
        }
        try {
            return el.getAsInt();
        } catch (Exception ignored) {
            if (el.isJsonPrimitive()) {
                try {
                    return Integer.parseInt(el.getAsString());
                } catch (Exception ignored2) {
                    return def;
                }
            }
            return def;
        }
    }

    private static String optString(JsonObject obj, String field) {
        if (obj == null || field == null) {
            return null;
        }
        JsonElement el = obj.get(field);
        if (el == null) {
            return null;
        }
        try {
            return el.getAsString();
        } catch (Exception ignored) {
            return null;
        }
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
