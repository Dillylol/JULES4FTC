# JULES (Just Useful Library for Easy Systems)

JULES is a lightweight, system-agnostic framework for FTC robots, designed to bridge the gap between robot hardware and external ML/AI pipelines.

## Getting Started

1.  **Standard Motor Naming**:
    Standardize your hardware configuration in the FTC Robot Controller app using these names:
    *   `lf`: Front Left Motor
    *   `rf`: Front Right Motor
    *   `lr`: Back Left Motor
    *   `rr`: Back Right Motor

2.  **Using JulesRobot**:
    Use the `JulesRobot` class in your OpModes for hardware initialization and basic drive control.
    ```java
    JulesRobot robot = new JulesRobot(hardwareMap, telemetry);
    robot.init();
    ```

3.  **JULES Bridge**:
    The bridge is enabled by default. You can see the WebSocket URL in the telemetry when your OpMode starts. Use this to connect with the JULES UI or external scripts.

## Project Structure

*   `core/`: Core JULES abstractions (`JulesRobot`).
*   `bridge/`: WebSocket and HTTP communication layer.
*   `constants/`: System-wide configuration (`JulesConstants`).
*   `examples/`: Sample OpModes to get you started.

## Dependencies

JULES requires the following dependencies in your `build.common.gradle`:
*   `NanoHTTPD` (Internal HTTP Server)
*   `NanoWSD` (WebSocket Support)
*   `OkHttp3` (Network Client)

---
*Maintained by JULES Team*
