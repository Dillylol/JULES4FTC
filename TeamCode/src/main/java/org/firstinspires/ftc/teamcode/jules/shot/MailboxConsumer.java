package org.firstinspires.ftc.teamcode.jules.shot;

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
    public ShotPlan poll(long nowMs) {
        String raw = JulesCommand.getAndClearCommand();
        if (raw == null) {
            return null;
        }
        JsonObject payload = ShotPlannerBridge.unwrap(raw);
        if (payload == null) {
            return null;
        }
        if (bridge.isModelUpdate(payload)) {
            rpmProvider.applyUpdate(payload);
            return null;
        }
        return bridge.tryParseShotPlan(payload, nowMs);
    }
}
