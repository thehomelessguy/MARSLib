---
name: marslib-led
description: Helps configure LED patterns, CANdle/AddressableLED wiring, and priority-based state rendering. Use when adding visual feedback, fault indicators, or alliance-colored patterns.
---

# MARSLib LED Feedback Skill

You are the visual feedback engineer for Team MARS 2614. When adding or modifying LED patterns:

## 1. Architecture

The LED system follows the standard IO abstraction pattern in `com.marslib.hmi`:

| Class | Purpose |
|---|---|
| `LEDIO` | IO interface — `setColor(r, g, b)`, `setPattern(pattern)` |
| `LEDIOAddressable` | WPILib AddressableLED implementation (PWM strip) |
| `LEDIOCANdle` | CTRE CANdle implementation (CAN-based LED controller) |
| `LEDManager` | Top-level subsystem — priority-based pattern selection |

### Dependency Injection
```java
// SIM / REPLAY mode: AddressableLED (software-only, no CAN)
ledManager = new LEDManager(new LEDIOAddressable(0, length), powerManager);

// REAL mode: CANdle hardware
ledManager = new LEDManager(new LEDIOCANdle(canId, canbus, length), powerManager);
```

### Priority Cascade
The LEDManager selects the active pattern based on priority (highest wins):
```
1. CRITICAL FAULT   → Red flash (MARSFaultManager active critical faults)
2. LOW VOLTAGE      → Amber pulse (MARSPowerManager below WARNING threshold)
3. GAME PIECE HELD  → Green solid (sensor confirms possession)
4. ALLIANCE IDLE    → Blue/Red based on DriverStation alliance
```

## 2. Key Rules

### Rule A: LEDManager Accepts MARSPowerManager
The LED subsystem participates in power shedding. During brownout conditions, the `MARSPowerManager` will shed LED power to preserve drive and mechanism actuators.

### Rule B: Hardware Wiring in IOReal Only
All physical CANdle API calls (animation, brightness, color) must be in `LEDIOCANdle`. The `LEDManager` only calls the generic `LEDIO` interface.

### Rule C: Pattern Timing Is Logged
LED state changes are logged via AdvantageKit:
```java
Logger.recordOutput("LED/CurrentPattern", pattern.name());
Logger.recordOutput("LED/FaultFlashActive", faultFlashActive);
```

### Rule D: Constants
```java
Constants.LEDConstants.CANDLE_ID = 24;
Constants.LEDConstants.CANBUS = "CAN2";
Constants.LEDConstants.LENGTH = 70;
```

## 3. Adding New Patterns
1. Add the pattern to the `LEDManager` priority logic
2. Ensure the pattern has a defined priority level (faults always highest)
3. Test with `LEDIOAddressable` in sim mode — no CAN required
4. Log the pattern name via AdvantageKit
