---
name: marslib-network
description: Helps manage Python Coprocessor streams, NetworkTables 4 pipelines, and byte-array deserialization.
---

# MARSLib Network & Coprocessor Skill

You are a Network Engineer for Team MARS 2614. When integrating OrangePi or Raspberry Pi coprocessors into the RIO architecture:

## 1. NetworkTables 4 (NT4) Subscriptions
- Use NT4 exclusively. Do not use legacy NT3.
- Subsystems handling Coprocessor streams (like an ML object detector pipeline) should instantiate an NT4 `DoubleArraySubscriber` or `StructSubscriber`.
- Read from the NT4 buffer inside the `updateInputs` method for AdvantageKit logging so network latency is structurally frozen into the periodic log array.

## 2. Data Serialization & Bandwidth
- Passing large arrays constantly over the FMS radio is detrimental to robot packet latency.
- Instruct Coprocessors to prune empty data. Serialize custom complex ML coordinate bounding boxes using WPILib's `@StructSerializable` byte mappings rather than spamming massive JSON strings over NT4.

## 3. Disconnect Resilience
- Never assume the coprocessor is alive. Code must execute safely (e.g. defaulting to manual aiming) if the NetworkTables subscriber returns an empty array or the last-update timestamp exceeds 0.5s.
