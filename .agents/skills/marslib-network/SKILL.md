---
name: marslib-network
description: Helps manage Python Coprocessor streams, NetworkTables 4 pipelines, and byte-array deserialization. Use when integrating coprocessors, reading NT4 data, or handling network resilience.
---

# MARSLib Network & Coprocessor Skill

You are a network engineer for Team MARS 2614. When integrating coprocessors or NetworkTables data:

## 1. Architecture

MARSLib uses NetworkTables 4 (NT4) exclusively for coprocessor communication:

| Component | Purpose |
|---|---|
| NT4 `DoubleArraySubscriber` | Receives numeric arrays from coprocessors (ML bounding boxes, etc.) |
| NT4 `StructSubscriber` | Receives WPILib struct data (Pose2d, etc.) from coprocessors |
| `@StructSerializable` | Binary serialization for bandwidth-efficient custom types |
| IO layer `updateInputs()` | Reads NT4 inside the AdvantageKit logging pipeline |

### Data Flow
```
Coprocessor (OrangePi/RPi) → NT4 Server → IO.updateInputs() → Logger.processInputs() → Subsystem
```

## 2. Key Rules

### Rule A: NT4 Only — No Legacy NT3
All NetworkTables communication MUST use NT4. Do not use legacy `NetworkTableInstance.getDefault().getTable()` patterns. Use typed subscribers (`DoubleArraySubscriber`, `StructSubscriber`) for type safety.

### Rule B: Read NT4 Inside updateInputs()
Coprocessor data MUST be read inside the IO interface's `updateInputs()` method, not in the subsystem's `periodic()`. This ensures network-sourced data is frozen into the AdvantageKit log alongside hardware data, enabling deterministic replay of coprocessor inputs.

### Rule C: Never Assume the Coprocessor is Alive
Always check the subscriber's last-update timestamp. If it exceeds 0.5s, fall back to manual/default behavior. A disconnected coprocessor must NEVER crash the robot or leave mechanisms in an unsafe state:
```java
if (Timer.getFPGATimestamp() - subscriber.getLastChange() > 0.5) {
    inputs.coprocessorConnected = false;
    return; // Fall back to defaults
}
```

### Rule D: Minimize Bandwidth Over FMS Radio
The FMS radio has limited bandwidth. Do NOT stream large JSON strings or high-frequency raw arrays. Use `@StructSerializable` binary mappings for complex types. Prune empty data on the coprocessor side. Limit update frequency to 10-20Hz for non-critical data.

## 3. Adding New Coprocessor Integrations

1. Create an IO interface (e.g., `ObjectDetectorIO.java`) with `@AutoLog` inputs.
2. Create the NT4 implementation (e.g., `ObjectDetectorIONT4.java`) that reads from subscribers in `updateInputs()`.
3. Create a sim implementation (e.g., `ObjectDetectorIOSim.java`) with synthetic test data.
4. In the subsystem, consume only the IO interface — never touch NT4 directly.
5. Add a timeout check for `coprocessorConnected` and implement fallback behavior.

## 4. Telemetry
- `{Coprocessor}/Connected` — Boolean: coprocessor is actively publishing
- `{Coprocessor}/Latency` — Time since last NT4 update
- `{Coprocessor}/FrameCount` — Cumulative frames received
- `{Coprocessor}/Data` — Latest processed data from the coprocessor
