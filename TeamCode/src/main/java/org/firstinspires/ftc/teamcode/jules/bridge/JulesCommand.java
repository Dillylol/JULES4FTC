// File: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/jules/bridge/JulesCommand.java
package org.firstinspires.ftc.teamcode.jules.bridge;

public class JulesCommand {

    private static volatile String currentCommand = null;

    /**
     * Sets the next command to be executed. This is called by the HTTP bridge.
     * 
     * @param command The full command string (e.g., "DRIVE_FORWARD_2T_0.5P").
     */
    public static synchronized void setCommand(String command) {
        if (command == null) {
            currentCommand = null;
        } else {
            currentCommand = command.trim();
        }
    }

    /**
     * Peeks at the current command without clearing it (for telemetry display).
     */
    public static synchronized String peekCommand() {
        return currentCommand;
    }

    /**
     * Gets the current command and immediately clears it to prevent re-execution.
     * This is called by the JulesDevController OpMode.
     * 
     * @return The command string, or null if no new command is present.
     */
    public static synchronized String getAndClearCommand() {
        String commandToReturn = currentCommand;
        currentCommand = null; // Clear the command after it's been retrieved
        return commandToReturn;
    }
}