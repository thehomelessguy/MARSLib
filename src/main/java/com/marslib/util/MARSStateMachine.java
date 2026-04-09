package com.marslib.util;

import edu.wpi.first.wpilibj.DataLogManager;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.Map;

/**
 * A generalized 2D Bilinear Interpolator for arbitrary math maps.
 *
 * <p>Top FRC teams utilize Bilinear Maps (Distance vs Velocity vs Output) to guarantee
 * deterministic, 100% precision scoring by interpolating over multi-variable constraints (e.g.,
 * Robot Speed AND Target Distance), bypassing algebraic regression errors entirely.
 */
public class MARSStateMachine<S extends Enum<S>> {
  private final Map<S, EnumSet<S>> validTransitions = new HashMap<>();
  private final Map<S, Runnable> entryActions = new HashMap<>();
  private final Map<S, Runnable> exitActions = new HashMap<>();
  private java.util.function.BiConsumer<S, S> onTransitionCallback = (from, to) -> {};

  private final Class<S> enumClass;
  private S currentState;
  private S previousState;
  private final String name;
  private int ticksInCurrentState;
  private int totalTransitionCount;

  public MARSStateMachine(String name, Class<S> enumClass, S initialState) {
    this.name = name;
    this.enumClass = enumClass;
    this.currentState = initialState;
    this.previousState = initialState;
    this.ticksInCurrentState = 0;
    this.totalTransitionCount = 0;
  }

  public void addTransition(S from, S to) {
    validTransitions.computeIfAbsent(from, k -> EnumSet.noneOf(enumClass)).add(to);
  }

  public void addValidTransition(S from, S to) {
    addTransition(from, to);
  }

  public void addValidBidirectional(S a, S b) {
    addValidTransition(a, b);
    addValidTransition(b, a);
  }

  public void addWildcardTo(S targetState) {
    for (S state : enumClass.getEnumConstants()) {
      addValidTransition(state, targetState);
    }
  }

  public void addWildcardFrom(S fromState) {
    for (S state : enumClass.getEnumConstants()) {
      addValidTransition(fromState, state);
    }
  }

  public boolean isTransitionLegal(S nextState) {
    if (currentState.equals(nextState)) return true;
    EnumSet<S> valid = validTransitions.get(currentState);
    return valid != null && valid.contains(nextState);
  }

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

  public S getState() {
    return currentState;
  }

  public int getTicksInCurrentState() {
    return ticksInCurrentState;
  }

  public S getPreviousState() {
    return previousState;
  }

  public int getTotalTransitionCount() {
    return totalTransitionCount;
  }

  public void setEntryAction(S state, Runnable action) {
    entryActions.put(state, action);
  }

  public void setExitAction(S state, Runnable action) {
    exitActions.put(state, action);
  }

  public void setOnTransition(java.util.function.BiConsumer<S, S> callback) {
    this.onTransitionCallback = callback;
  }

  public void update() {
    ticksInCurrentState++;
  }
}
