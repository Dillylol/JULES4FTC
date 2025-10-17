package org.firstinspires.ftc.teamcode.jules.link;

import androidx.annotation.NonNull;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.ThreadFactory;
import java.util.concurrent.TimeUnit;

/**
 * Single-threaded scheduler for Jules link tasks.
 */
public final class JulesScheduler {

    private final ScheduledExecutorService executor;
    private final List<ScheduledFuture<?>> futures = new ArrayList<>();

    public JulesScheduler() {
        this.executor = Executors.newSingleThreadScheduledExecutor(new ThreadFactory() {
            @Override
            public Thread newThread(@NonNull Runnable r) {
                Thread thread = new Thread(r, "JulesLinkScheduler");
                thread.setDaemon(true);
                return thread;
            }
        });
    }

    public synchronized ScheduledFuture<?> scheduleAtFixedRate(Runnable task, long initialDelayMs, long periodMs) {
        ScheduledFuture<?> future = executor.scheduleAtFixedRate(task, initialDelayMs, periodMs, TimeUnit.MILLISECONDS);
        futures.add(future);
        return future;
    }

    public synchronized void shutdown() {
        for (ScheduledFuture<?> future : futures) {
            if (future != null) {
                future.cancel(true);
            }
        }
        futures.clear();
        executor.shutdownNow();
    }
}

