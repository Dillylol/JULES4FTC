// File: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/jules/bridge/JulesBuffer.java

package org.firstinspires.ftc.teamcode.jules.bridge;

import org.firstinspires.ftc.teamcode.jules.Metrics;
import com.qualcomm.robotcore.util.ElapsedTime; // Import the timer
import java.util.concurrent.atomic.AtomicInteger;

public class JulesBuffer {
    private final Metrics[] buf;
    private final AtomicInteger writeIdx = new AtomicInteger(0);
    private volatile int count = 0;
    private final ElapsedTime timer; // Add a timer for timestamps

    public JulesBuffer(int capacity) {
        this.buf = new Metrics[capacity];
        this.timer = new ElapsedTime(); // Initialize the timer
    }

    public void push(Metrics m) {
        int i = writeIdx.getAndIncrement();
        // This logic handles wrapping the circular buffer index
        if (i >= buf.length) {
            // This CAS (compare-and-set) is a thread-safe way to reset the index to 1
            // if another thread hasn't already done so. It prevents a race condition.
            writeIdx.compareAndSet(i + 1, 1);
            i = 0;
        }
        buf[i] = m;
        if (count < buf.length) count++;
    }

    /**
     * Injects a labeled marker into the data buffer.
     * This creates a special Metrics object that will appear in data dumps.
     * @param text The label text for the marker.
     */
    public void label(String text) {
        Metrics marker = new Metrics();
        marker.t = timer.seconds(); // Use the buffer's timer for a consistent timestamp
        marker.label = text;
        push(marker); // Add the marker to the buffer
    }

    /** Returns a copy of the buffer's data, ordered from oldest to newest. */
    public Metrics[] snapshot() {
        Metrics[] out = new Metrics[count];
        int w = writeIdx.get();
        int n = count;

        // This logic correctly handles reading from the circular buffer
        // It starts from the oldest element and reads up to the newest.
        for (int k = 0; k < n; k++) {
            int readHead = w - n;
            int idx = (readHead + k) % buf.length;
            if (idx < 0) {
                idx += buf.length;
            }
            out[k] = buf[idx];
        }
        return out;
    }

    public void clear() {
        writeIdx.set(0);
        count = 0;
    }

    public int size() {
        return count;
    }

    public int capacity() {
        return buf.length;
    }
}