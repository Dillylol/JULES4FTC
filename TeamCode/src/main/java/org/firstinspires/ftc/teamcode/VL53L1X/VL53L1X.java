package org.firstinspires.ftc.teamcode.VL53L1X;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.TypeConversion;

@I2cDeviceType
@DeviceProperties(name = "VL53L1X Time of Flight", xmlTag = "VL53L1X", description = "VL53L1X ToF Sensor")
public class VL53L1X extends I2cDeviceSynchDevice<I2cDeviceSynch> {

    public final static I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create7bit(0x29);

    // --- Registers (Partial Map from VL53L1X.h) ---
    private static final int SOFT_RESET = 0x0000;
    private static final int I2C_SLAVE__DEVICE_ADDRESS = 0x0001;
    private static final int OSC_MEASURED__FAST_OSC__FREQUENCY = 0x0006;
    private static final int VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND = 0x0008;
    private static final int VHV_CONFIG__INIT = 0x000B;
    private static final int ALGO__CROSSTALK_COMPENSATION_VALID_HEIGHT_MM = 0x0039;
    private static final int ALGO__RANGE_IGNORE_VALID_HEIGHT_MM = 0x003E;
    private static final int ALGO__RANGE_MIN_CLIP = 0x003F;
    private static final int ALGO__CONSISTENCY_CHECK__TOLERANCE = 0x0040;
    private static final int PHASECAL_CONFIG__TIMEOUT_MACROP = 0x004B;
    private static final int PHASECAL_CONFIG__OVERRIDE = 0x004D;
    private static final int DSS_CONFIG__ROI_MODE_CONTROL = 0x004F;
    private static final int SYSTEM__THRESH_RATE_HIGH = 0x0050;
    private static final int SYSTEM__THRESH_RATE_LOW = 0x0052;
    private static final int DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT = 0x0054;
    private static final int DSS_CONFIG__APERTURE_ATTENUATION = 0x0057;
    private static final int MM_CONFIG__TIMEOUT_MACROP_A = 0x005A;
    private static final int MM_CONFIG__TIMEOUT_MACROP_B = 0x005C;
    private static final int RANGE_CONFIG__TIMEOUT_MACROP_A = 0x005E;
    private static final int RANGE_CONFIG__VCSEL_PERIOD_A = 0x0060;
    private static final int RANGE_CONFIG__TIMEOUT_MACROP_B = 0x0061;
    private static final int RANGE_CONFIG__VCSEL_PERIOD_B = 0x0063;
    private static final int RANGE_CONFIG__SIGMA_THRESH = 0x0064;
    private static final int RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS = 0x0066;
    private static final int RANGE_CONFIG__VALID_PHASE_HIGH = 0x0069;
    private static final int SYSTEM__INTERMEASUREMENT_PERIOD = 0x006C;
    private static final int SYSTEM__GROUPED_PARAMETER_HOLD_0 = 0x0071;
    private static final int SYSTEM__SEED_CONFIG = 0x0077;
    private static final int SD_CONFIG__WOI_SD0 = 0x0078;
    private static final int SD_CONFIG__WOI_SD1 = 0x0079;
    private static final int SD_CONFIG__INITIAL_PHASE_SD0 = 0x007A;
    private static final int SD_CONFIG__INITIAL_PHASE_SD1 = 0x007B;
    private static final int SYSTEM__GROUPED_PARAMETER_HOLD_1 = 0x007C;
    private static final int SD_CONFIG__QUANTIFIER = 0x007E;
    private static final int SYSTEM__SEQUENCE_CONFIG = 0x0081;
    private static final int SYSTEM__GROUPED_PARAMETER_HOLD = 0x0082;
    private static final int FIRMWARE__SYSTEM_STATUS = 0x00E5;
    private static final int SYSTEM__INTERRUPT_CLEAR = 0x0086;
    private static final int SYSTEM__MODE_START = 0x0087;
    private static final int RESULT__RANGE_STATUS = 0x0089;
    private static final int RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0 = 0x008C;
    private static final int RESULT__AMBIENT_COUNT_RATE_MCPS_SD0 = 0x0090;
    private static final int RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0 = 0x0096;
    private static final int RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0 = 0x0098;
    private static final int ALGO__PART_TO_PART_RANGE_OFFSET_MM = 0x001E;
    private static final int MM_CONFIG__OUTER_OFFSET_MM = 0x0022;
    private static final int PAD_I2C_HV__EXTSUP_CONFIG = 0x002E;
    private static final int GPIO__TIO_HV_STATUS = 0x0031;
    private static final int SIGMA_ESTIMATOR__EFFECTIVE_PULSE_WIDTH_NS = 0x0036;
    private static final int SIGMA_ESTIMATOR__EFFECTIVE_AMBIENT_WIDTH_NS = 0x0037;
    private static final int PHASECAL_RESULT__VCSEL_START = 0x00D8;
    private static final int RESULT__OSC_CALIBRATE_VAL = 0x00DE;
    private static final int CAL_CONFIG__VCSEL_START = 0x0047;
    private static final int IDENTIFICATION__MODEL_ID = 0x010F;
    private static final int DSS_CONFIG__TARGET_TOTAL_RATE_MCPS = 0x0024;

    private static final int TARGET_RATE = 0x0A00;
    private static final int TIMING_GUARD = 4528;

    // State variables
    private int fastOscFrequency;
    private int oscCalibrateVal;
    private boolean calibrated = false;
    private byte savedVhvInit;
    private byte savedVhvTimeout;
    private long timeoutStartMs;
    private int ioTimeout = 0;

    // Result Buffer
    public class RangingData {
        public int rangeMm;
        public int rangeStatus;
        public float peakSignalCountRateMcps;
        public float ambientCountRateMcps;
    }

    public RangingData rangingData = new RangingData();

    public enum DistanceMode {
        Short, Medium, Long, Unknown
    }

    private DistanceMode currentDistanceMode = DistanceMode.Unknown;

    public VL53L1X(I2cDeviceSynch deviceClient) {
        super(deviceClient, true);
        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);
        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    @Override
    protected boolean doInitialize() {
        return initSensor();
    }

    // Ported from VL53L1X::init
    private boolean initSensor() {
        // Soft Reset
        writeReg(SOFT_RESET, (byte) 0x00);
        try {
            Thread.sleep(1);
        } catch (InterruptedException e) {
        }
        writeReg(SOFT_RESET, (byte) 0x01);
        try {
            Thread.sleep(1);
        } catch (InterruptedException e) {
        }

        // Wait for Boot
        long startTime = System.currentTimeMillis();
        while ((readReg(FIRMWARE__SYSTEM_STATUS) & 0x01) == 0) {
            if (System.currentTimeMillis() - startTime > 500)
                return false; // Timeout
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
            }
        }

        // 2V8 Mode (Standard for most modules)
        writeReg(PAD_I2C_HV__EXTSUP_CONFIG, (byte) (readReg(PAD_I2C_HV__EXTSUP_CONFIG) | 0x01));

        // Store Oscillator Info
        fastOscFrequency = readReg16Short(OSC_MEASURED__FAST_OSC__FREQUENCY) & 0xFFFF;
        oscCalibrateVal = readReg16Short(RESULT__OSC_CALIBRATE_VAL) & 0xFFFF;

        // Static Config
        writeReg16(DSS_CONFIG__TARGET_TOTAL_RATE_MCPS, (short) TARGET_RATE);
        writeReg(GPIO__TIO_HV_STATUS, (byte) 0x02);
        writeReg(SIGMA_ESTIMATOR__EFFECTIVE_PULSE_WIDTH_NS, (byte) 8);
        writeReg(SIGMA_ESTIMATOR__EFFECTIVE_AMBIENT_WIDTH_NS, (byte) 16);
        writeReg(ALGO__CROSSTALK_COMPENSATION_VALID_HEIGHT_MM, (byte) 0x01);
        writeReg(ALGO__RANGE_IGNORE_VALID_HEIGHT_MM, (byte) 0xFF);
        writeReg(ALGO__RANGE_MIN_CLIP, (byte) 0);
        writeReg(ALGO__CONSISTENCY_CHECK__TOLERANCE, (byte) 2);

        // General Config
        writeReg16(SYSTEM__THRESH_RATE_HIGH, (short) 0x0000);
        writeReg16(SYSTEM__THRESH_RATE_LOW, (short) 0x0000);
        writeReg(DSS_CONFIG__APERTURE_ATTENUATION, (byte) 0x38);

        // Timing Config
        writeReg16(RANGE_CONFIG__SIGMA_THRESH, (short) 360);
        writeReg16(RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS, (short) 192);

        // Dynamic Config
        writeReg(SYSTEM__GROUPED_PARAMETER_HOLD_0, (byte) 0x01);
        writeReg(SYSTEM__GROUPED_PARAMETER_HOLD_1, (byte) 0x01);
        writeReg(SD_CONFIG__QUANTIFIER, (byte) 2);

        writeReg(SYSTEM__GROUPED_PARAMETER_HOLD, (byte) 0x00);
        writeReg(SYSTEM__SEED_CONFIG, (byte) 1);

        // Low Power Auto Mode Config
        writeReg(SYSTEM__SEQUENCE_CONFIG, (byte) 0x8B); // VHV, PHASECAL, DSS1, RANGE
        writeReg16(DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, (short) (200 << 8));
        writeReg(DSS_CONFIG__ROI_MODE_CONTROL, (byte) 2); // REQUESTED_EFFFECTIVE_SPADS

        // Default to Long Range, 50ms
        setDistanceMode(DistanceMode.Long);
        setMeasurementTimingBudget(50000);

        // Offset Calibration
        writeReg16(ALGO__PART_TO_PART_RANGE_OFFSET_MM, (short) (readReg16Short(MM_CONFIG__OUTER_OFFSET_MM) * 4));

        startContinuous(50);
        return true;
    }

    public void startContinuous(int periodMs) {
        writeReg32(SYSTEM__INTERMEASUREMENT_PERIOD, periodMs * oscCalibrateVal);
        writeReg(SYSTEM__INTERRUPT_CLEAR, (byte) 0x01);
        writeReg(SYSTEM__MODE_START, (byte) 0x40); // Timed Mode
    }

    // Software Calibration Parameters
    // Tuned 2026-01-26 Based on user data (avg 8.75" overshoot)
    // Derived from: Real = Measured * 0.973 - 176mm, correcting previous (1.0157x +
    // 8.16)
    private double correctionAlpha = 0.988;
    private double correctionBeta = -168.0;

    /**
     * Set the linear correction parameters.
     * Formula: Result = (Raw * alpha) + beta
     * 
     * @param alpha Slope/Gain (default 1.0)
     * @param beta  Intercept/Offset in mm (default 0)
     */
    public void setCorrectionParams(double alpha, double beta) {
        this.correctionAlpha = alpha;
        this.correctionBeta = beta;
    }

    public int getDistanceMm() {
        int raw = read(false);
        if (raw < 0 || raw == 65535)
            return raw; // Pass through error/invalid codes
        return (int) (raw * correctionAlpha + correctionBeta);
    }

    public int getRawDistanceMm() {
        return read(false);
    }

    // Core Read Logic from VL53L1X::read
    public int read(boolean blocking) {
        if (blocking) {
            long start = System.currentTimeMillis();
            while ((readReg(GPIO__TIO_HV_STATUS) & 0x01) != 0) { // Wait while bit is 1 (Not Ready)
                if (System.currentTimeMillis() - start > 500)
                    return -1;
            }
        } else {
            if ((readReg(GPIO__TIO_HV_STATUS) & 0x01) != 0) // If bit is 1, return -1 (Not Ready)
                return -1;
        }

        // Read Results
        readResultsBuffer();

        if (!calibrated) {
            setupManualCalibration();
            calibrated = true;
        }

        updateDSS();

        // Calculate Range
        int range = rangingData.rangeMm; // Logic handled in readResultsBuffer/update methods?
        // No, C++ calls getRangingData().

        // Calculate final range
        int rawRange = readReg16Short(RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0) & 0xFFFF;
        // Correction gain: (range * 2011 + 0x0400) / 0x0800
        int finalRange = (int) ((long) rawRange * 2011 + 0x0400) / 0x0800;

        // Clear Interrupt
        writeReg(SYSTEM__INTERRUPT_CLEAR, (byte) 0x01);

        return finalRange;
    }

    private void updateDSS() {
        int spadCount = readReg16Short(RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0) & 0xFFFF;
        if (spadCount != 0) {
            int totalRatePerSpad = (readReg16Short(RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0)
                    & 0xFFFF) +
                    (readReg16Short(RESULT__AMBIENT_COUNT_RATE_MCPS_SD0) & 0xFFFF);
            if (totalRatePerSpad > 0xFFFF)
                totalRatePerSpad = 0xFFFF;
            totalRatePerSpad <<= 16;
            totalRatePerSpad /= spadCount;
            if (totalRatePerSpad != 0) {
                int requiredSpads = (TARGET_RATE << 16) / totalRatePerSpad;
                if (requiredSpads > 0xFFFF)
                    requiredSpads = 0xFFFF;
                writeReg16(DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, (short) requiredSpads);
                return;
            }
        }
        writeReg16(DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, (short) 0x8000);
    }

    private void setupManualCalibration() {
        savedVhvInit = readReg(VHV_CONFIG__INIT);
        savedVhvTimeout = readReg(VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND);
        writeReg(VHV_CONFIG__INIT, (byte) (savedVhvInit & 0x7F));
        writeReg(VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, (byte) ((savedVhvTimeout & 0x03) + (3 << 2)));
        writeReg(PHASECAL_CONFIG__OVERRIDE, (byte) 0x01);
        writeReg(CAL_CONFIG__VCSEL_START, readReg(PHASECAL_RESULT__VCSEL_START));
    }

    // Placeholder, real read does a burst read. kept simple here.
    private void readResultsBuffer() {
        // Just ensures we access the regs logic correctly
    }

    public void setDistanceMode(DistanceMode mode) {
        // save budget
        // int budget = getMeasurementTimingBudget(); // Todo

        switch (mode) {
            case Short:
                writeReg(RANGE_CONFIG__VCSEL_PERIOD_A, (byte) 0x07);
                writeReg(RANGE_CONFIG__VCSEL_PERIOD_B, (byte) 0x05);
                writeReg(RANGE_CONFIG__VALID_PHASE_HIGH, (byte) 0x38);
                writeReg(SD_CONFIG__WOI_SD0, (byte) 0x07);
                writeReg(SD_CONFIG__WOI_SD1, (byte) 0x05);
                writeReg(SD_CONFIG__INITIAL_PHASE_SD0, (byte) 6);
                writeReg(SD_CONFIG__INITIAL_PHASE_SD1, (byte) 6);
                break;
            case Medium:
                writeReg(RANGE_CONFIG__VCSEL_PERIOD_A, (byte) 0x0B);
                writeReg(RANGE_CONFIG__VCSEL_PERIOD_B, (byte) 0x09);
                writeReg(RANGE_CONFIG__VALID_PHASE_HIGH, (byte) 0x78);
                writeReg(SD_CONFIG__WOI_SD0, (byte) 0x0B);
                writeReg(SD_CONFIG__WOI_SD1, (byte) 0x09);
                writeReg(SD_CONFIG__INITIAL_PHASE_SD0, (byte) 10);
                writeReg(SD_CONFIG__INITIAL_PHASE_SD1, (byte) 10);
                break;
            case Long:
                writeReg(RANGE_CONFIG__VCSEL_PERIOD_A, (byte) 0x0F);
                writeReg(RANGE_CONFIG__VCSEL_PERIOD_B, (byte) 0x0D);
                writeReg(RANGE_CONFIG__VALID_PHASE_HIGH, (byte) 0xB8);
                writeReg(SD_CONFIG__WOI_SD0, (byte) 0x0F);
                writeReg(SD_CONFIG__WOI_SD1, (byte) 0x0D);
                writeReg(SD_CONFIG__INITIAL_PHASE_SD0, (byte) 14);
                writeReg(SD_CONFIG__INITIAL_PHASE_SD1, (byte) 14);
                break;
            default:
                return;
        }
        currentDistanceMode = mode;
        // setMeasurementTimingBudget(budget); // Re-apply
    }

    public boolean setMeasurementTimingBudget(int budgetUs) {
        if (budgetUs <= TIMING_GUARD)
            return false;
        int usedBudget = budgetUs - TIMING_GUARD;
        if (usedBudget > 1100000)
            return false;
        usedBudget /= 2;

        int macroPeriodUs = calcMacroPeriod(readReg(RANGE_CONFIG__VCSEL_PERIOD_A));
        int phasecalTimeoutMclks = timeoutMicrosecondsToMclks(1000, macroPeriodUs);
        if (phasecalTimeoutMclks > 0xFF)
            phasecalTimeoutMclks = 0xFF;
        writeReg(PHASECAL_CONFIG__TIMEOUT_MACROP, (byte) phasecalTimeoutMclks);

        writeReg16(MM_CONFIG__TIMEOUT_MACROP_A, (short) encodeTimeout(timeoutMicrosecondsToMclks(1, macroPeriodUs)));
        writeReg16(RANGE_CONFIG__TIMEOUT_MACROP_A,
                (short) encodeTimeout(timeoutMicrosecondsToMclks(usedBudget, macroPeriodUs)));

        macroPeriodUs = calcMacroPeriod(readReg(RANGE_CONFIG__VCSEL_PERIOD_B));
        writeReg16(MM_CONFIG__TIMEOUT_MACROP_B, (short) encodeTimeout(timeoutMicrosecondsToMclks(1, macroPeriodUs)));
        writeReg16(RANGE_CONFIG__TIMEOUT_MACROP_B,
                (short) encodeTimeout(timeoutMicrosecondsToMclks(usedBudget, macroPeriodUs)));

        return true;
    }

    // --- Helpers from CPP ---
    private int calcMacroPeriod(int vcsel_period) {
        int pll_period_us = (readReg(0x0104) << 24) | (readReg(0x0105) << 16) | (readReg(0x0106) << 8)
                | readReg(0x0107); // PLL_PERIOD_US
        // Wait, C++ says:
        // uint8_t PLL_PERIOD_US = 0x0104;
        // It reads 32bit from there? Yes.
        // Actually java readReg is byte.
        // Let's implement read32.
        // But for now, macro_period_us logic:
        // C++: calcMacroPeriod logic is complex?
        // "PLL_PERIOD_US * vcsel_period_clocks"
        // (vcsel_period_pclks = (vcsel_period + 1) << 1 ?)
        // Use simplified:
        // Let's implement full logic:
        int vcsel_period_pclks = (vcsel_period + 1) << 1;
        // read 32 bit pll period
        int mp = readReg32(0x0104);
        int macro_period_v = (int) ((long) mp * vcsel_period_pclks);
        return macro_period_v >> 6;
    }

    private int timeoutMicrosecondsToMclks(int timeout_us, int macro_period_us) {
        return (int) (((long) timeout_us * 1000 + (macro_period_us / 2)) / macro_period_us);
    }

    private int encodeTimeout(int timeout_mclks) {
        int ls_byte = 0;
        int ms_byte = 0;
        if (timeout_mclks > 0) {
            ls_byte = timeout_mclks - 1;
            while ((ls_byte & 0xFFFFFF00) > 0) {
                ls_byte >>= 1;
                ms_byte++;
            }
            return (ms_byte << 8) | (ls_byte & 0xFF);
        }
        return 0;
    }

    // --- I2C Helpers ---
    public void writeReg(int reg, byte value) {
        this.deviceClient.write((reg >> 8) & 0xFF, new byte[] { (byte) (reg & 0xFF), value });
    }

    public void writeReg16(int reg, short value) {
        this.deviceClient.write((reg >> 8) & 0xFF,
                new byte[] { (byte) (reg & 0xFF), (byte) ((value >> 8) & 0xFF), (byte) (value & 0xFF) });
    }

    public void writeReg32(int reg, int value) {
        this.deviceClient.write((reg >> 8) & 0xFF, new byte[] { (byte) (reg & 0xFF),
                (byte) ((value >> 24) & 0xFF), (byte) ((value >> 16) & 0xFF), (byte) ((value >> 8) & 0xFF),
                (byte) (value & 0xFF) });
    }

    public byte readReg(int reg) {
        this.deviceClient.write((reg >> 8) & 0xFF, new byte[] { (byte) (reg & 0xFF) });
        byte[] data = this.deviceClient.read(1);
        return data.length > 0 ? data[0] : 0;
    }

    public short readReg16Short(int reg) {
        this.deviceClient.write((reg >> 8) & 0xFF, new byte[] { (byte) (reg & 0xFF) });
        byte[] data = this.deviceClient.read(2);
        return TypeConversion.byteArrayToShort(data);
    }

    public int readReg32(int reg) {
        this.deviceClient.write((reg >> 8) & 0xFF, new byte[] { (byte) (reg & 0xFF) });
        byte[] data = this.deviceClient.read(4);
        return TypeConversion.byteArrayToInt(data);
    }

    public byte readReg16Byte(int reg) {
        return readReg(reg);
    }

    public int getModelID() {
        return readReg16Short(IDENTIFICATION__MODEL_ID) & 0xFFFF;
    }

    public boolean isConnected() {
        return getModelID() == 0xEACC;
    }

    /**
     * Set the calibration offset in millimeters.
     * The sensor adds this value to the measured distance.
     * 
     * @param offsetMm The offset in millimeters (can be negative).
     */
    public void setCalibrationOffset(int offsetMm) {
        // Register expects value * 4 (fixed point 14.2)
        writeReg16(ALGO__PART_TO_PART_RANGE_OFFSET_MM, (short) (offsetMm * 4));
    }

    /**
     * Get the current calibration offset in millimeters.
     * 
     * @return The offset in millimeters.
     */
    public int getCalibrationOffset() {
        // Register value is * 4, so divide by 4 to get mm
        short rawOffset = readReg16Short(ALGO__PART_TO_PART_RANGE_OFFSET_MM);
        return rawOffset / 4;
    }

    @Override
    public HardwareDevice.Manufacturer getManufacturer() {
        return HardwareDevice.Manufacturer.STMicroelectronics;
    }

    @Override
    public String getDeviceName() {
        return "VL53L1X Time of Flight";
    }
}
