ODrive S1 via CAN – ros2\_control Hardware Interface
====================================================

Engineering Requirements (with mandatory tests)
-----------------------------------------------

### 1\. Scope and Objectives

**Component:** odrive\_control::OdriveS1CanSystem implementing hardware\_interface::SystemInterface.

**Goals**

1.  Control N ODrive S1 axes over CAN via ros2\_control as a single system:
    
    *   Command interfaces: position, velocity, effort (mapped to current/torque).
        
    *   State interfaces: position, velocity, effort plus diagnostics.
        
2.  Support revolute and prismatic joints, with optional URDF elements for gear and lead-screw mapping.
    
3.  Integrate with:
    
    *   controller\_manager, joint\_trajectory\_controller, standard broadcasters.
        
    *   MoveIt2 for multi-DOF arms and gantries.
        
4.  Provide robust fault handling, diagnostics, and ODrive-specific health reporting.
    
5.  Optional but well-specified: startup limit-consistency checks between URDF/joint limits and ODrive firmware limits.
    

**Testing requirement:**Every numbered functional requirement in this document MUST have:

*   At least one **unit test** verifying the behavior of the responsible code paths in isolation.
    
*   At least one **integration test** verifying the behavior in a composed ROS 2/ros2\_control environment (with real or simulated CAN).
    

### 2\. System Context and Topology

*   Single ROS 2 process hosting:
    
    *   controller\_manager node.
        
    *   The OdriveS1CanSystem hardware plugin.
        
*   One CAN bus (e.g. can0) shared by:
    
    *   A USB-CAN adapter on the host.
        
    *   M ODrive S1 boards, each with up to 2 axes.
        

**Requirements**

2.1 The hardware plugin MUST represent the entire robot as one SystemInterface instance.2.2 The plugin MUST support an arbitrary mapping of URDF joints to (node\_id, axis\_index) pairs.2.3 Configuration MUST allow multiple ODrives on one CAN bus and up to 2 axes per ODrive.

**Testing**

*   Unit: mapping logic from joint config → internal axis indices.
    
*   Integration: bring up a composed system with ≥2 ODrives (or simulated endpoints) and verify that commands for N joints route to the correct CAN IDs/axes.
    

### 3\. Joint Modeling and Transmissions

#### 3.1 URDF / ros2\_control

3.1.1 Joints MUST be modeled in URDF as:

*   revolute / continuous with positions in radians.
    
*   prismatic with positions in meters.
    

3.1.2 Under :

*   Each joint MUST declare:
    
    *   command\_interface entries for position, velocity, effort as needed.
        
    *   state\_interface entries for position, velocity, effort.
        

3.1.3 The hardware plugin MUST support:

*   Operation without tags (joint-space only).
    
*   Operation with tags using transmission\_interface (e.g. SimpleTransmission) for actuator↔joint mapping.
    

#### 3.2 Internal representations

3.2.1 Internally, state and commands MUST be represented in actuator space (e.g. turns, turns/s, A or Nm).3.2.2 read() MUST convert actuator state → joint state using:

*   Transmissions, if defined.
    
*   Otherwise, configured gear/lead-screw ratios.3.2.3 write() MUST convert joint commands → actuator commands using the same mapping, ensuring consistency.
    

**Testing**

*   Unit:
    
    *   Deterministic mapping tests for revolute and prismatic joints with and without transmissions.
        
    *   Edge cases (zero reduction, invalid config) must be handled predictably (error, exception, or clear log).
        
*   Integration:
    
    *   Load a URDF with mixed joint types and transmissions, run the hardware in a test node, and verify joint state and command values via ROS topics reflect expected actuator behavior (with a CAN stub).
        

### 4\. CAN / ODrive S1 Interface Layer

#### 4.1 Transport

4.1.1 System MUST support a configurable CAN interface name (e.g. can0).4.1.2 CAN bitrate MUST be configurable and validated at startup.4.1.3 Each ODrive board MUST be addressed by a unique CAN node ID; axes MUST be correctly distinguished.

#### 4.2 Protocol

4.2.1 The plugin MUST support CANSimple:

*   Axis state control (Set\_Axis\_State).
    
*   Motion commands (Set\_Input\_Pos, Set\_Input\_Vel, Set\_Input\_Torque or equivalent).
    
*   State feedback (position, velocity, current, error words) via cyclical messages.
    
*   Heartbeats for liveness and axis state/error monitoring.
    

4.2.2 A minimal RT-safe abstraction MUST encapsulate CAN operations:

*   Non-blocking or bounded blocking in real-time sections.
    
*   Clear separation between RT read()/write() and any non-RT configuration/SDO operations.
    

**Testing**

*   Unit:
    
    *   CAN frame encoding/decoding for all supported commands and responses.
        
    *   Conversion between internal AxisCommand/AxisState structures and CAN frames.
        
*   Integration:
    
    *   Use vCAN + a simulated ODrive server to verify end-to-end command and heartbeat interaction (commands sent, state updated, errors reflected).
        

### 5\. Control Modes and Command Interfaces

5.1 Supported control modes for each joint:

*   Position control via position command → ODrive position input.
    
*   Velocity control via velocity command → ODrive velocity input.
    
*   Effort/current control via effort command → ODrive torque/current input.
    

5.2 At any given time, for each joint exactly one of these command interfaces MUST be active.

5.3 The plugin MUST implement prepare\_command\_mode\_switch() and perform\_command\_mode\_switch() such that:

*   Illegal interface combinations (e.g. simultaneous position and velocity commands to same joint) are rejected.
    
*   Mode switches are atomic per control update and drive axes are transitioned cleanly (e.g. enabling/disabling CLOSED\_LOOP state as needed).
    

**Testing**

*   Unit:
    
    *   Mode switching state machine, including rejection paths for invalid combinations.
        
*   Integration:
    
    *   Start/stop different controllers (pos vs vel) and verify:
        
        *   Mode switches are accepted/rejected as expected.
            
        *   Drive behavior (in simulation or on hardware) transitions correctly (IDLE ↔ CLOSED\_LOOP).
            

### 6\. Read/Write Semantics and Timing

6.1 The standard control loop order MUST be respected: read() → controllers → write().6.2 read() MUST:

*   Use the latest AxisState data for each axis.
    
*   Convert into joint units and update state interfaces.
    

6.3 write() MUST:

*   Compute actuator commands in actuator units.
    
*   Send minimal CAN commands (skipping unchanged commands within tolerance).
    
*   Respect configurable limits on CAN utilization.
    

6.4 End-to-end latency from joint command to CAN transmission SHOULD be ≤ 2 control cycles under nominal conditions.

**Testing**

*   Unit:
    
    *   read() and write() with mocked CAN layer.
        
*   Integration:
    
    *   Time-stamped tests to check command → CAN frame timing and verify that state reflects changes within expected latency bounds.
        

### 7\. Fault and Error Handling

7.1 The plugin MUST continuously monitor ODrive error fields:

*   axis\_error, motor\_error, controller\_error, encoder\_error, axis\_state.
    

7.2 It MUST maintain per-axis fault states with categories:

*   OK
    
*   WARNING (degraded or threshold approaching)
    
*   ERROR (axis disarmed, major fault)
    

7.3 When an ERROR is detected:

*   The affected joint MUST be marked as faulted.
    
*   Commands for that joint MUST NOT be sent until explicit recovery.
    
*   ODrive MAY be commanded to IDLE to ensure no motion.
    

7.4 Errors MUST be reflected in:

*   HardwareStatus (per-joint device state).
    
*   /diagnostics output.
    

7.5 The system MUST expose an explicit recovery path:

*   Service/API to clear ODrive errors.
    
*   Re-arm axis and re-enter CLOSED\_LOOP only on explicit request.
    

**Testing**

*   Unit:
    
    *   Error word → severity mapping.
        
    *   Transition logic (OK→ERROR→OK) for a single axis.
        
*   Integration:
    
    *   Inject synthetic error frames and verify:
        
        *   Joint is marked faulted.
            
        *   Commands cease.
            
        *   Status/diagnostics reflect the fault.
            
    *   Simulate recovery sequence and verify normal operation resumes.
        

### 8\. Homing and Calibration

8.1 The system MUST support ODrive-side homing (endstops/index) per axis, triggered via ROS (service/action/command).8.2 On homing success:

*   Joint zero MUST be aligned with the intended URDF zero, considering transmissions and offsets.
    

8.3 On homing failure:

*   The joint MUST be marked as faulted.
    
*   No trajectories MUST be accepted for that joint until resolved.
    

**Testing**

*   Unit:
    
    *   Homing state machine and success/failure classification.
        
*   Integration:
    
    *   Simulated homing success/failure scenarios and checking:
        
        *   Joint position is reset correctly.
            
        *   Fault behavior on failure is correct.
            

### 9\. Diagnostics and HardwareStatus

9.1 The plugin MUST implement the HardwareStatus extension hooks so that the framework can publish control\_msgs/msg/HardwareStatus at a configurable rate.9.2 For each joint, HardwareStatus MUST include:

*   Current health status (OK/WARNING/ERROR).
    
*   Operational mode (e.g. IDLE, CLOSED\_LOOP, HOMING).
    
*   Power state (ON/OFF/ERROR).
    
*   State details with key/value info (errors, last heartbeat age, etc.).
    

9.3 The plugin MUST also publish diagnostics on /diagnostics using diagnostic\_updater, summarizing:

*   Global health.
    
*   Per-axis key metrics and error states.
    

**Testing**

*   Unit:
    
    *   HardwareStatus population logic from internal AxisState + fault state.
        
    *   Diagnostic updater callbacks.
        
*   Integration:
    
    *   Bring up the system, induce known conditions (healthy, warning, error) and verify messages on /hardware\_status and /diagnostics are correct.
        

### 10\. Configuration and Parameters

10.1 System-level parameters MUST include:

*   can\_interface (string)
    
*   status\_publish\_rate (double Hz)
    
*   Default control mode
    
*   Timeouts (CAN RX, heartbeat)
    
*   Logging verbosity levels.
    

10.2 Per-joint parameters MUST include:

*   odrive\_node\_id
    
*   odrive\_axis\_index
    
*   Optional gear\_ratio, lead\_screw\_pitch, torque\_constant when not using transmissions.
    

10.3 All parameters MUST be validated at configuration time, and invalid configs MUST fail configuration with clear error messages.

**Testing**

*   Unit:
    
    *   Parameter parsing and validation logic.
        
*   Integration:
    
    *   Launch with valid and invalid parameter sets and verify:
        
        *   Correct acceptance or rejection.
            
        *   Clear error logs on misconfiguration.
            

### 11\. Optional: Limit Consistency Checks (URDF vs ODrive Config)

11.1 An optional feature MUST be implemented (but may be disabled) to compare joint limits in URDF/joint\_limits.yaml with ODrive firmware limits over CAN.

11.2 Configuration:

*   limits\_check.mode REQUIRED; values:
    
    *   OFF: feature disabled.
        
    *   WARN\_ONLY: log and report mismatches, still start.
        
    *   STRICT: mismatches cause configuration failure.
        
*   Tolerance ratios for velocity, effort/current, and acceleration MUST be configurable.
    

11.3 Behavior:

*   At configure() or on\_activate():
    
    *   Read relevant ODrive parameters via CAN (e.g. vel limit, current limit).
        
    *   Map URDF joint limits to actuator units via transmissions/ratios.
        
    *   Compare per axis per limit type using configured tolerances.
        
*   On mismatch:
    
    *   In WARN\_ONLY: log, mark diagnostic WARNING, note in HardwareStatus.
        
    *   In STRICT: treat as configuration error and prevent startup.
        

11.4 No automatic modification of ODrive config is allowed; checks MUST be read-only.

**Testing**

*   Unit:
    
    *   Mapping and comparison logic, including borderline cases around tolerance thresholds.
        
*   Integration:
    
    *   Simulated ODrive endpoints with:
        
        *   Matching limits (no warnings).
            
        *   Slightly lower limits (warnings).
            
        *   Significantly lower limits (STRICT mode → configuration failure).
            

### 12\. MoveIt2 Integration

12.1 The hardware MUST work with joint\_trajectory\_controller as the primary command controller:

*   Position or velocity trajectories as configured.
    

12.2 Joint states MUST be broadcast via joint\_state\_broadcaster using the hardware state interfaces.

12.3 MoveIt2 planning with the corresponding SRDF and controllers MUST:

*   Execute trajectories without systematic violations of internal limits (assuming URDF and ODrive configs are consistent).
    
*   Respect goal tolerances and not exhibit persistent drift or oscillation under nominal conditions.
    

**Testing**

*   Integration:
    
    *   Test workspace with a simple MoveIt2 configuration (2–3 DOF simulated robot) using the ODrive hardware plugin against a simulated ODrive server:
        
        *   Execute random or canonical trajectories.
            
        *   Ensure trajectories complete without hardware faults and with expected joint endpoint errors.
            

### 13\. Testing Strategy Summary (Mandatory)

13.1 The project MUST include:

*   A dedicated **unit test suite** (e.g. gtest) that:
    
    *   Runs in CI.
        
    *   Covers:
        
        *   Mapping logic (transmissions, joint↔actuator).
            
        *   Mode switching.
            
        *   Fault classification and transitions.
            
        *   Limit check calculations.
            
        *   Configuration parsing and validation.
            
*   A dedicated **integration test suite** that:
    
    *   Runs in CI or a separate hardware-in-the-loop target.
        
    *   Uses vCAN and an ODrive simulation harness at minimum.
        
    *   Optionally includes hardware-in-the-loop tests for final validation.
        

13.2 All new features and bug fixes MUST be accompanied by unit and, where applicable, integration tests demonstrating:

*   The bug is reproduced without the fix.
    
*   The fix resolves the bug without breaking existing tests.
    

13.3 The minimum bar for merge MUST be:

*   All unit tests pass.
    
*   All integration tests pass.
    
*   No untested public behavior changes (any behavior described in these requirements must be covered by at least one test).