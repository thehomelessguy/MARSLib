package com.marslib.util;

import edu.wpi.first.wpilibj.DataLogManager;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;

/**
 * A validated, enum-based Finite State Machine supporting guarded transitions, entry/exit actions,
 * and transition callbacks.
 *
 * <p>This FSM enforces an explicit transition table: only pre-registered transitions are allowed.
 * Illegal transition requests are logged and rejected, preventing undefined mechanism states that
 * could cause collisions or damage. The FSM tracks ticks spent in each state for timeout-based
 * logic.
 *
 * <p>Students: Use this class to coordinate multi-step mechanisms (e.g., intake → score sequences).
 * Register allowed transitions with {@link #addValidTransition}, then call {@link
 * #requestTransition} to move between states safely.
 *
 * @param <S> The enum type representing the set of valid states.
 */
public class MARSStateMachine<S extends Enum<S>> {
  private final Map<S, EnumSet<S>> validTransitions = new HashMap<>();
  private final Map<S, Runnable> entryActions = new HashMap<>();
  private final Map<S, Runnable> exitActions = new HashMap<>();
  private BiConsumer<S, S> onTransitionCallback = (from, to) -> {};

  private final Class<S> enumClass;
  private S currentState;
  private S previousState;
  private final String name;
  private int ticksInCurrentState;
  private int totalTransitionCount;

  /**
   * Constructs a new state machine.
   *
   * @param name Human-readable name for log output (e.g., "Superstructure").
   * @param enumClass The enum class token for the state type (e.g., {@code MyState.class}).
   * @param initialState The state the machine starts in.
   */
  public MARSStateMachine(String name, Class<S> enumClass, S initialState) {
    this.name = name;
    this.enumClass = enumClass;
    this.currentState = initialState;
    this.previousState = initialState;
    this.ticksInCurrentState = 0;
    this.totalTransitionCount = 0;
  }

  /**
   * Registers a one-directional transition from {@code from} to {@code to}.
   *
   * @param from The source state.
   * @param to The destination state.
   */
  public void addTransition(S from, S to) {
    validTransitions.computeIfAbsent(from, k -> EnumSet.noneOf(enumClass)).add(to);
  }

  /**
   * Alias for {@link #addTransition(Enum, Enum)}.
   *
   * @param from The source state.
   * @param to The destination state.
   */
  public void addValidTransition(S from, S to) {
    addTransition(from, to);
  }

  /**
   * Registers transitions in both directions between {@code a} and {@code b}.
   *
   * @param a The first state.
   * @param b The second state.
   */
  public void addValidBidirectional(S a, S b) {
    addValidTransition(a, b);
    addValidTransition(b, a);
  }

  /**
   * Makes {@code targetState} reachable from every state in the enum.
   *
   * @param targetState The state that all others can transition to.
   */
  public void addWildcardTo(S targetState) {
    for (S state : enumClass.getEnumConstants()) {
      addValidTransition(state, targetState);
    }
  }

  /**
   * Makes {@code fromState} able to transition to every state in the enum.
   *
   * @param fromState The state that can go anywhere.
   */
  public void addWildcardFrom(S fromState) {
    for (S state : enumClass.getEnumConstants()) {
      addValidTransition(fromState, state);
    }
  }

  /**
   * Checks whether a transition from the current state to {@code nextState} would be accepted,
   * without actually performing it.
   *
   * @param nextState The proposed destination state.
   * @return {@code true} if the transition is legal (including self-transitions).
   */
  public boolean isTransitionLegal(S nextState) {
    if (currentState.equals(nextState)) return true;
    EnumSet<S> valid = validTransitions.get(currentState);
    return valid != null && valid.contains(nextState);
  }

  /**
   * Attempts to transition from the current state to {@code nextState}. If the transition is legal,
   * exit actions, the transition callback, and entry actions fire in sequence and the state
   * changes. Self-transitions are always accepted but do NOT fire any actions. Illegal transitions
   * are logged and rejected.
   *
   * @param nextState The desired state.
   * @return {@code true} if the transition was accepted, {@code false} if rejected.
   */
  public boolean requestTransition(S nextState) {
    if (currentState.equals(nextState)) {
      return true;
    }

    if (isTransitionLegal(nextState)) {
      DataLogManager.log("[FSM " + name + "] Transitioning: " + currentState + " -> " + nextState);

      if (exitActions.containsKey(currentState)) {
        exitActions.get(currentState).run();
      }

      onTransitionCallback.accept(currentState, nextState);
      previousState = currentState;
      currentState = nextState;
      ticksInCurrentState = 0;
      totalTransitionCount++;

      if (entryActions.containsKey(nextState)) {
        entryActions.get(nextState).run();
      }
      return true;
    } else {
      DataLogManager.log(
          "[FSM ERROR "
              + name
              + "] ILLEGAL TRANSITION ATTEMPTED: "
              + currentState
              + " -> "
              + nextState);
      return false; // BLOCKED!
    }
  }

  /**
   * Returns the current active state.
   *
   * @return The current state enum value.
   */
  public S getState() {
    return currentState;
  }

  /**
   * Returns the number of ticks since the last transition.
   *
   * @return Tick count in the current state (incremented by {@link #update()}).
   */
  public int getTicksInCurrentState() {
    return ticksInCurrentState;
  }

  /**
   * Returns the state that was active before the most recent transition.
   *
   * @return The previous state enum value.
   */
  public S getPreviousState() {
    return previousState;
  }

  /**
   * Returns the total number of accepted (non-self) transitions since construction.
   *
   * @return Cumulative transition count.
   */
  public int getTotalTransitionCount() {
    return totalTransitionCount;
  }

  /**
   * Registers a callback to run when entering the specified state.
   *
   * @param state The state whose entry triggers the action.
   * @param action The callback to execute on entry.
   */
  public void setEntryAction(S state, Runnable action) {
    entryActions.put(state, action);
  }

  /**
   * Registers a callback to run when exiting the specified state.
   *
   * @param state The state whose exit triggers the action.
   * @param action The callback to execute on exit.
   */
  public void setExitAction(S state, Runnable action) {
    exitActions.put(state, action);
  }

  /**
   * Registers a callback that fires on every accepted transition, receiving the source and
   * destination states.
   *
   * @param callback A {@link BiConsumer} that receives (fromState, toState).
   */
  public void setOnTransition(BiConsumer<S, S> callback) {
    this.onTransitionCallback = callback;
  }

  /**
   * Increments the tick counter for the current state. Must be called once per subsystem periodic
   * loop — if omitted, {@link #getTicksInCurrentState()} will not advance and timeout-based logic
   * will break.
   */
  public void update() {
    ticksInCurrentState++;
  }
}
