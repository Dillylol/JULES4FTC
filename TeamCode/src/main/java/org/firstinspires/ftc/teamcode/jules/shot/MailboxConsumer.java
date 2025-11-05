package org.firstinspires.ftc.teamcode.jules.shot;

import androidx.annotation.Nullable;

import com.google.gson.JsonObject;

import org.firstinspires.ftc.teamcode.jules.bridge.JulesCommand;

/**
 * Polls the shared {@link JulesCommand} mailbox and routes planner messages.
 */
public final class MailboxConsumer {

    private final ShotPlannerBridge bridge;
    private final RpmProvider rpmProvider;

    public MailboxConsumer(ShotPlannerBridge bridge, RpmProvider rpmProvider) {
        this.bridge = bridge;
        this.rpmProvider = rpmProvider;
    }

    /**
     * Poll the mailbox once. Returns a new {@link ShotPlan} when accepted; otherwise {@code null}.
     */
    public ShotPlan poll(long wallClockNowMs) {
        while (true) {
            JsonObject payload = bridge != null ? bridge.pollInbound() : null;
            if (payload == null) {
                String raw = JulesCommand.getAndClearCommand();
                if (raw == null || bridge == null) {
                    return null;
                }
                payload = ShotPlannerBridge.unwrap(raw);
                if (payload == null) {
                    continue;
                }
            }

            if (bridge.isModelUpdate(payload)) {
                rpmProvider.applyUpdate(payload);
                continue;
            }

            if (bridge.isShotResult(payload)) {
                continue;
            }

            ShotPlan plan = bridge.tryParseShotPlan(payload, wallClockNowMs);
            if (plan != null) {
                return plan;
            }
        }
    }

    public ShotPlannerBridge.ShotResult pollShotResult() {
        return bridge != null ? bridge.pollShotResult() : null;
    }

    public ShotPlannerBridge.HelloAck pollHelloAck() {
        return bridge != null ? bridge.pollHelloAck() : null;
    }
}
