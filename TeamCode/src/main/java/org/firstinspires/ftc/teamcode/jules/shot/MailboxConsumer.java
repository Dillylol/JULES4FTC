package org.firstinspires.ftc.teamcode.jules.shot;

import com.google.gson.JsonObject;

import org.firstinspires.ftc.teamcode.jules.bridge.JulesCommand;

/**
 * Polls the shared {@link JulesCommand} mailbox for planner messages.
 * Accepts both plain JSON objects and wrapped command envelopes.
 */
public final class MailboxConsumer {
    private final ShotPlannerBridge bridge;

    public interface Handler {
        void onShotPlan(ShotPlan plan);
        void onRpmModelUpdate(JsonObject update);
    }

    public MailboxConsumer(ShotPlannerBridge bridge) {
        this.bridge = bridge;
    }

    public void poll(long nowMs, Handler handler) {
        if (handler == null) {
            return;
        }
        String raw = JulesCommand.getAndClearCommand();
        if (raw == null || raw.trim().isEmpty()) {
            return;
        }
        handleRaw(nowMs, handler, raw);
    }

    private void handleRaw(long nowMs, Handler handler, String raw) {
        if (raw == null || raw.trim().isEmpty()) {
            return;
        }
        String candidate = bridge.unwrapCommandEnvelope(raw.trim());
        if (candidate == null || candidate.isEmpty()) {
            return;
        }
        ShotPlan plan = bridge.tryParseShotPlan(candidate);
        if (plan != null) {
            plan.setReceivedAtMs(nowMs);
            handler.onShotPlan(plan);
            return;
        }
        JsonObject rpmUpdate = bridge.tryParseRpmModelUpdate(candidate);
        if (rpmUpdate != null) {
            handler.onRpmModelUpdate(rpmUpdate);
        }
    }
}
