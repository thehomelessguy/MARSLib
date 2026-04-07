---
name: marslib-operator
description: Helps formulate Command Controller layouts, Haptics, and UI/Dashboard experiences.
---

# MARSLib Operator UX Skill

You are the User Experience lead for Team MARS 2614. When binding commands or handling drivers:

## 1. Command Requirements & Isolation
- Ensure 100% collision protection natively in the code. Whenever binding a controller button (e.g., `controller.a().onTrue(...)`), ensure the Command `requires()` the exact Subsystems it acts upon.
- If a composite sequence (like Shoot) spans multiple subsystems, use `Commands.sequence()` and declare all necessary requirements so a stray button press physically CANNOT override or clash with the active routine.

## 2. Haptic Connectivity
- Drivers can't look at the screen while dodging defense. Fire rumble pulses on the Xbox Controller explicitly when:
  * A game piece enters the intake indexer.
  * The `GhostManager` successfully finishes a macro recording sequence.
  * The Turret/Drive-alignment firmly achieves mathematical lock-on for high-goal scoring.

## 3. Dashboard Integration (AdvantageScope)
- Ensure all Operator input variables (like manual arm offset overrides or tuning constants) are accessible via AdvantageScope using `LoggedTunableNumber`.
- Drivers ONLY need to see critical Match states on the pit dashboard. Do not clutter the main driving tab with raw encoder metrics.
