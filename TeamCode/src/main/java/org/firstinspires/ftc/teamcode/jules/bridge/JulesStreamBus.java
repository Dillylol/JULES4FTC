package org.firstinspires.ftc.teamcode.jules.bridge;

import java.io.Closeable;
import java.util.Set;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.LinkedBlockingQueue;

/**
 * Generic line-based stream bus.
 * - Call publishJsonLine("{...}") for live samples (one JSON object per call).
 * - Clients can subscribe() to receive those lines in order.
 * - No formatting here: upstream encoders produce the JSON strings.
 */
public final class JulesStreamBus implements AutoCloseable {

    public static final class Subscription implements Closeable {
        private final BlockingQueue<String> q = new LinkedBlockingQueue<>();
        private volatile boolean open = true;
        private final JulesStreamBus bus;

        private Subscription(JulesStreamBus bus) { this.bus = bus; }

        /** Called by bus */
        void offer(String line) {
            if (open) q.offer(line);
        }

        /** Client: take next line (blocking). Returns null if closed. */
        public String take() throws InterruptedException {
            while (open) {
                String s = q.poll();
                if (s != null) return s;
                Thread.sleep(5);
            }
            return null;
        }

        /** Client: take next line (non-blocking). Returns null if empty or closed. */
        public String poll() {
            if (!open) return null;
            return q.poll();
        }

        @Override public void close() {
            open = false;
            bus.drop(this);
        }
    }

    private final Set<Subscription> subs = ConcurrentHashMap.newKeySet();
    private volatile boolean open = true;

    /** Publish a single JSON line (no trailing newline needed). */
    public void publishJsonLine(String jsonLine) {
        if (!open || jsonLine == null || jsonLine.isEmpty()) return;
        for (Subscription s : subs) s.offer(jsonLine);
    }

    /** Create a new subscription. Caller must close it. */
    public Subscription subscribe() {
        Subscription s = new Subscription(this);
        subs.add(s);
        return s;
    }

    void drop(Subscription s) { subs.remove(s); }

    @Override public void close() {
        open = false;
        for (Subscription s : subs) s.close();
        subs.clear();
    }
}
