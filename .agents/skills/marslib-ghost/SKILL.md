---
name: marslib-ghost
description: Helps configure and extend the GhostManager teleop replay system. Use when recording driver macros, configuring playback commands, or debugging replay-autonomy integration.
---

# MARSLib Ghost Replay Skill

You are a replay systems engineer for Team MARS 2614. When working with teleop recording or autonomous playback macros:

## 1. Architecture

`GhostManager` (`com.marslib.auto`) enables recording and replaying driver inputs for autonomous:

| Class | Purpose |
|---|---|
| `GhostManager` | Central controller — writes/reads CSV macro files, wires button + joystick state |
| `TeleopDriveMath` | Pure-function joystick→ChassisSpeeds math, shared between live and playback |

### Recording Flow
1. Pilot presses `Back + Start` simultaneously → triggers `registerRecordCommand()`
2. Every 20ms loop, GhostManager serializes: `leftY`, `leftX`, `rightX`, and 12 button booleans
3. Data is enqueued to a `ConcurrentLinkedQueue<String>` to avoid blocking the main loop
4. On command end: background thread flushes the queue to `AutoConstants.GHOST_MACRO_FILE_PATH` (CSV)

### Playback Flow
1. `getPlaybackCommand()` returns a `Command` registered as an auto option
2. During execution, GhostManager reads CSV rows at 20ms intervals
3. Interceptor methods redirect joystick calls: `getLeftY(fallback)` returns recorded value during playback, or live `fallback` during teleop
4. The drive pipeline (`TeleopDriveMath`) processes the data identically to live driving

## 2. Key Rules

### Rule A: Ghost Intercepts Joystick Methods
All joystick reads in `configureDefaultCommands()` go through ghost interceptors:
```java
ghostManager.getLeftY(() -> controller.getLeftY())
ghostManager.getLeftX(() -> controller.getLeftX())
ghostManager.getRightX(() -> controller.getRightX())
```
During recording/playback, these return stored values. During normal teleop, they return the live lambda.

### Rule B: File I/O Is Async
Macro writes use `ConcurrentLinkedQueue` to buffer data. The CSV flush happens on command `end()`, NOT during `execute()`. This prevents file I/O from blocking the 20ms loop.

### Rule C: Ghost Macro Is an Auto Option
GhostManager playback is registered in `autoChooser`:
```java
autoChooser.addDefaultOption("Ghost Playback", ghostManager.getPlaybackCommand());
```
This means the recording from the most recent ghost session can be replayed autonomously.

### Rule D: Button State Recording
Ghost records ALL 12 button booleans (A, B, X, Y, LB, RB, DPad U/D/L/R) plus 3 axes. When replaying, only the joystick axes are replayed — button commands are NOT auto-triggered (the mechanism state machine is driven separately by autonomous).

## 3. Telemetry
- `Ghost/Recording` — Boolean: currently recording
- `Ghost/Playing` — Boolean: currently replaying
- `Ghost/FrameCount` — Number of frames recorded/played
- `Ghost/FilePath` — Path to the macro CSV file
