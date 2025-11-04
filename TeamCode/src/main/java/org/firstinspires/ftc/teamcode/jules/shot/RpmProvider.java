package org.firstinspires.ftc.teamcode.jules.shot;

import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;

import java.util.ArrayList;
import java.util.List;

/**
 * Computes flywheel RPM targets based on distance, voltage, and optional delta bins.
 */
public final class RpmProvider {
    private static final double MIN_VOLTAGE = 10.5;
    private static final double MAX_VOLTAGE = 13.0;
    private static final int MIN_RPM = 0;
    private static final int MAX_RPM = 6000;

    private double mRpmPerFt;
    private double bRpmOffset;
    private double vNom;
    private double gamma;
    private List<DeltaBin> deltaBins;

    public RpmProvider(double mRpmPerFt, double bRpmOffset, double vNom, double gamma) {
        this.mRpmPerFt = mRpmPerFt;
        this.bRpmOffset = bRpmOffset;
        this.vNom = vNom;
        this.gamma = gamma;
        this.deltaBins = new ArrayList<>();
    }

    public static RpmProvider withDefaults() {
        return new RpmProvider(116.4042383594456, 2084.2966941424975, 12.5, 1.0);
    }

    public synchronized int targetRpm(double distanceIn, double vBatt) {
        double distanceFeet = Math.max(0.0, distanceIn) / 12.0;
        double base = (mRpmPerFt * distanceFeet) + bRpmOffset;
        double voltage = clamp(vBatt, MIN_VOLTAGE, MAX_VOLTAGE);
        double scale = (vNom > 0 && gamma != 0) ? Math.pow(voltage / vNom, gamma) : 1.0;
        double rpm = base * scale;
        rpm += computeDelta(distanceIn, voltage);
        rpm = clamp(rpm, MIN_RPM, MAX_RPM);
        return (int) Math.round(rpm);
    }

    public synchronized void applyUpdate(JsonObject update) {
        if (update == null) {
            return;
        }
        try {
            if (update.has("m_rpm_per_ft") && update.get("m_rpm_per_ft").isJsonPrimitive()) {
                mRpmPerFt = update.get("m_rpm_per_ft").getAsDouble();
            }
            if (update.has("b_rpm_offset") && update.get("b_rpm_offset").isJsonPrimitive()) {
                bRpmOffset = update.get("b_rpm_offset").getAsDouble();
            }
            if (update.has("v_nom") && update.get("v_nom").isJsonPrimitive()) {
                vNom = update.get("v_nom").getAsDouble();
            }
            if (update.has("gamma") && update.get("gamma").isJsonPrimitive()) {
                gamma = update.get("gamma").getAsDouble();
            }
            if (update.has("delta_bins") && update.get("delta_bins").isJsonArray()) {
                parseDeltaBins(update.getAsJsonArray("delta_bins"));
            }
        } catch (Exception ignored) {
            // Ignore malformed fields per contract.
        }
    }

    private void parseDeltaBins(JsonArray array) {
        List<DeltaBin> bins = new ArrayList<>();
        for (JsonElement element : array) {
            if (!element.isJsonObject()) {
                continue;
            }
            JsonObject obj = element.getAsJsonObject();
            double[] dRange = parseRange(obj.get("d"));
            double[] vRange = parseRange(obj.get("v"));
            if (dRange == null || vRange == null) {
                continue;
            }
            int delta = obj.has("delta") && obj.get("delta").isJsonPrimitive()
                    ? obj.get("delta").getAsInt()
                    : 0;
            bins.add(new DeltaBin(dRange[0], dRange[1], vRange[0], vRange[1], delta));
        }
        synchronized (this) {
            deltaBins = bins;
        }
    }

    private static double[] parseRange(JsonElement element) {
        if (element == null) {
            return null;
        }
        if (element.isJsonArray()) {
            JsonArray arr = element.getAsJsonArray();
            if (arr.size() >= 2 && arr.get(0).isJsonPrimitive() && arr.get(1).isJsonPrimitive()) {
                double lo = arr.get(0).getAsDouble();
                double hi = arr.get(1).getAsDouble();
                if (Double.isNaN(lo) || Double.isNaN(hi)) {
                    return null;
                }
                if (lo > hi) {
                    double tmp = lo;
                    lo = hi;
                    hi = tmp;
                }
                return new double[]{lo, hi};
            }
        }
        return null;
    }

    private double computeDelta(double distanceIn, double vBatt) {
        double delta = 0.0;
        for (DeltaBin bin : deltaBins) {
            if (bin.matches(distanceIn, vBatt)) {
                delta += bin.delta;
            }
        }
        return delta;
    }

    private static double clamp(double value, double lo, double hi) {
        return Math.max(lo, Math.min(hi, value));
    }

    private static final class DeltaBin {
        private final double dMin;
        private final double dMax;
        private final double vMin;
        private final double vMax;
        private final int delta;

        private DeltaBin(double dMin, double dMax, double vMin, double vMax, int delta) {
            this.dMin = dMin;
            this.dMax = dMax;
            this.vMin = vMin;
            this.vMax = vMax;
            this.delta = delta;
        }

        private boolean matches(double distanceIn, double vBatt) {
            return distanceIn >= dMin && distanceIn <= dMax && vBatt >= vMin && vBatt <= vMax;
        }
    }
}
