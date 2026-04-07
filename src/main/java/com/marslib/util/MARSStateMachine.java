package com.marslib.util;

import java.util.EnumMap;
import java.util.EnumSet;
import java.util.Map;
import java.util.function.BiConsumer;
import org.littletonrobotics.junction.Logger;

/**
 * A generic, reusable finite state machine with transition validation, logging, and action hooks.
 *
 * <p>This provides a proper FSM abstraction for coordinating complex subsystem behavior — elevator
 * + arm collision avoidance, intake sequencing, etc. Unlike raw switch-case blocks, this class:
 *
 * <ul>
 *   <li>Validates that every transition is explicitly declared legal (illegal transitions are
 *       rejected and logged).
 *   <li>Fires optional entry/exit/transition actions for side effects.
 *   <li>Logs every state change to AdvantageKit with timestamps for post-match debugging.
 *   <li>Tracks the number of ticks spent in each state for cycle-time analysis.
 * </ul>
 *
 * <p><b>Usage:</b>
 *
 * <pre>{@code
 * MARSStateMachine<MyState> machine = new MARSStateMachine<>("MySubsystem", MyState.class, MyState.IDLE);
 * machine.addTransition(MyState.IDLE, MyState.ACTIVE);
 * machine.addTransition(MyState.ACTIVE, MyState.IDLE);
 * machine.setOnTransition((from, to) -> Logger.recordOutput("Transition", from + "→" + to));
 *
 * // In periodic():
 * machine.update(); // logs tick count, checks for timed states, etc.
 * }</pre>
 *
 * @param <S> An enum type representing the set of discrete states.
 */
public class MARSStateMachine<S extends Enum<S>> {

  private final String name;
  private final Class<S> stateClass;
  private S currentState;
  private S previousState;
  private int ticksInCurrentState = 0;
  private int totalTransitionCount = 0;

  /**
   * Adjacency map: for each state, the set of states it is allowed to transition to. If a state has
   * no entry, ALL transitions from it are illegal.
   */
  private final Map<S, EnumSet<S>> legalTransitions;

  /** Optional callback fired on every valid transition (from, to). */
  private BiConsumer<S, S> onTransition;

  /** Optional per-state entry actions. */
  private final Map<S, Runnable> entryActions;

  /** Optional per-state exit actions. */
  private final Map<S, Runnable> exitActions;

  /**
   * Creates a new state machine.
   *
   * @param name Human-readable name for logging (e.g., "Superstructure").
   * @param stateClass The enum class (e.g., {@code MyState.class}).
   * @param initialState The starting state.
   */
  public MARSStateMachine(String name, Class<S> stateClass, S initialState) {
    this.name = name;
    this.stateClass = stateClass;
    this.currentState = initialState;
    this.previousState = initialState;
    this.legalTransitions = new EnumMap<>(stateClass);
    this.entryActions = new EnumMap<>(stateClass);
    this.exitActions = new EnumMap<>(stateClass);
  }

  /**
   * Declares a legal transition from one state to another. Transitions not declared here will be
   * rejected by {@link #requestTransition(Enum)}.
   *
   * @param from Source state.
   * @param to Destination state.
   * @return This machine (for chaining).
   */
  public MARSStateMachine<S> addTransition(S from, S to) {
    legalTransitions.computeIfAbsent(from, k -> EnumSet.noneOf(stateClass)).add(to);
    return this;
  }

  /**
   * Declares that the given state can transition to ALL other states. Useful for emergency or reset
   * states like STOWED.
   *
   * @param from Source state that can go anywhere.
   * @return This machine (for chaining).
   */
  public MARSStateMachine<S> addWildcardFrom(S from) {
    legalTransitions.put(from, EnumSet.allOf(stateClass));
    return this;
  }

  /**
   * Declares that ALL states can transition to the given target. Useful for emergency stow or
   * e-stop.
   *
   * @param to Target state reachable from anywhere.
   * @return This machine (for chaining).
   */
  public MARSStateMachine<S> addWildcardTo(S to) {
    for (S state : EnumSet.allOf(stateClass)) {
      legalTransitions.computeIfAbsent(state, k -> EnumSet.noneOf(stateClass)).add(to);
    }
    return this;
  }

  /**
   * Sets a callback invoked on every valid state transition.
   *
   * @param onTransition A consumer receiving (fromState, toState).
   */
  public void setOnTransition(BiConsumer<S, S> onTransition) {
    this.onTransition = onTransition;
  }

  /**
   * Sets an action to run when entering the specified state.
   *
   * @param state The target state.
   * @param action The action to run on entry.
   */
  public void setEntryAction(S state, Runnable action) {
    entryActions.put(state, action);
  }

  /**
   * Sets an action to run when exiting the specified state.
   *
   * @param state The source state.
   * @param action The action to run on exit.
   */
  public void setExitAction(S state, Runnable action) {
    exitActions.put(state, action);
  }

  /**
   * Requests a transition to the given target state. If the transition is not declared legal, it is
   * rejected and logged — the current state does not change.
   *
   * @param target The desired next state.
   * @return {@code true} if the transition was accepted, {@code false} if rejected.
   */
  public boolean requestTransition(S target) {
    if (target == currentState) {
      return true; // Self-transition is always a no-op
    }

    EnumSet<S> allowed = legalTransitions.get(currentState);
    if (allowed == null || !allowed.contains(target)) {
      // Illegal transition — log and reject
      Logger.recordOutput(name + "/RejectedTransition", currentState.name() + "→" + target.name());
      Logger.recordOutput(
          name + "/RejectedTransitionTimestamp", edu.wpi.first.wpilibj.Timer.getFPGATimestamp());
      return false;
    }

    // Legal transition — execute
    S from = currentState;
    previousState = from;

    // Fire exit action
    Runnable exitAction = exitActions.get(from);
    if (exitAction != null) {
      exitAction.run();
    }

    currentState = target;
    ticksInCurrentState = 0;
    totalTransitionCount++;

    // Fire entry action
    Runnable entryAction = entryActions.get(target);
    if (entryAction != null) {
      entryAction.run();
    }

    // Fire transition callback
    if (onTransition != null) {
      onTransition.accept(from, target);
    }

    // Log the transition
    Logger.recordOutput(name + "/Transition", from.name() + "→" + target.name());
    Logger.recordOutput(
        name + "/TransitionTimestamp", edu.wpi.first.wpilibj.Timer.getFPGATimestamp());

    return true;
  }

  /** Must be called every periodic loop. Updates tick counters and logs state telemetry. */
  public void update() {
    ticksInCurrentState++;
    Logger.recordOutput(name + "/CurrentState", currentState.name());
    Logger.recordOutput(name + "/TicksInState", ticksInCurrentState);
    Logger.recordOutput(name + "/TotalTransitions", totalTransitionCount);
  }

  /** Returns the current state. */
  public S getState() {
    return currentState;
  }

  /** Returns the previous state (before the last transition). */
  public S getPreviousState() {
    return previousState;
  }

  /** Returns the number of ticks spent in the current state. */
  public int getTicksInCurrentState() {
    return ticksInCurrentState;
  }

  /** Returns the total number of valid transitions since construction. */
  public int getTotalTransitionCount() {
    return totalTransitionCount;
  }

  /**
   * Checks if a transition from the current state to the given target is legal.
   *
   * @param target The candidate state.
   * @return {@code true} if the transition is declared legal.
   */
  public boolean isTransitionLegal(S target) {
    if (target == currentState) return true;
    EnumSet<S> allowed = legalTransitions.get(currentState);
    return allowed != null && allowed.contains(target);
  }
}
