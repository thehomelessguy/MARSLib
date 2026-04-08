package com.marslib.util;

import static org.junit.jupiter.api.Assertions.*;

import com.marslib.testing.MARSTestHarness;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Tests for the generic {@link MARSStateMachine} framework. Validates transition validation,
 * illegal transition rejection, entry/exit actions, and tick counting.
 */
public class MARSStateMachineTest {

  private enum TestState {
    IDLE,
    ACTIVE,
    SCORING,
    EMERGENCY
  }

  private MARSStateMachine<TestState> machine;

  @BeforeEach
  public void setUp() {
    MARSTestHarness.reset();
    machine = new MARSStateMachine<>("Test", TestState.class, TestState.IDLE);

    // Define transitions: IDLE→ACTIVE, ACTIVE→SCORING, SCORING→IDLE
    // EMERGENCY is reachable from anywhere (wildcard-to)
    machine.addTransition(TestState.IDLE, TestState.ACTIVE);
    machine.addTransition(TestState.ACTIVE, TestState.SCORING);
    machine.addTransition(TestState.SCORING, TestState.IDLE);
    machine.addWildcardTo(TestState.EMERGENCY);
    machine.addTransition(TestState.EMERGENCY, TestState.IDLE);
  }

  @Test
  public void testInitialState() {
    assertEquals(TestState.IDLE, machine.getState());
    assertEquals(0, machine.getTotalTransitionCount());
  }

  @Test
  public void testLegalTransition() {
    boolean accepted = machine.requestTransition(TestState.ACTIVE);
    assertTrue(accepted, "IDLE→ACTIVE should be legal");
    assertEquals(TestState.ACTIVE, machine.getState());
    assertEquals(TestState.IDLE, machine.getPreviousState());
    assertEquals(1, machine.getTotalTransitionCount());
  }

  @Test
  public void testIllegalTransitionRejected() {
    // IDLE→SCORING is not declared, should be rejected
    boolean accepted = machine.requestTransition(TestState.SCORING);
    assertFalse(accepted, "IDLE→SCORING should be illegal");
    assertEquals(TestState.IDLE, machine.getState(), "State should not have changed");
    assertEquals(0, machine.getTotalTransitionCount());
  }

  @Test
  public void testSelfTransitionIsNoOp() {
    boolean accepted = machine.requestTransition(TestState.IDLE);
    assertTrue(accepted, "Self-transition should be accepted as no-op");
    assertEquals(TestState.IDLE, machine.getState());
    assertEquals(0, machine.getTotalTransitionCount(), "Self-transition shouldn't count");
  }

  @Test
  public void testWildcardToReachableFromAnywhere() {
    // EMERGENCY should be reachable from IDLE
    assertTrue(machine.requestTransition(TestState.EMERGENCY));
    assertEquals(TestState.EMERGENCY, machine.getState());

    // Reset to IDLE, go to ACTIVE, then EMERGENCY
    machine.requestTransition(TestState.IDLE);
    machine.requestTransition(TestState.ACTIVE);
    assertTrue(machine.requestTransition(TestState.EMERGENCY));
    assertEquals(TestState.EMERGENCY, machine.getState());
  }

  @Test
  public void testChainedTransitions() {
    assertTrue(machine.requestTransition(TestState.ACTIVE));
    assertTrue(machine.requestTransition(TestState.SCORING));
    assertTrue(machine.requestTransition(TestState.IDLE));
    assertEquals(TestState.IDLE, machine.getState());
    assertEquals(3, machine.getTotalTransitionCount());
  }

  @Test
  public void testTickCounting() {
    assertEquals(0, machine.getTicksInCurrentState());

    machine.update();
    machine.update();
    machine.update();
    assertEquals(3, machine.getTicksInCurrentState());

    // Transition resets tick counter
    machine.requestTransition(TestState.ACTIVE);
    assertEquals(0, machine.getTicksInCurrentState());

    machine.update();
    assertEquals(1, machine.getTicksInCurrentState());
  }

  @Test
  public void testEntryAndExitActions() {
    int[] entryCount = {0};
    int[] exitCount = {0};

    machine.setEntryAction(TestState.ACTIVE, () -> entryCount[0]++);
    machine.setExitAction(TestState.IDLE, () -> exitCount[0]++);

    machine.requestTransition(TestState.ACTIVE);

    assertEquals(1, exitCount[0], "Exit action for IDLE should have fired");
    assertEquals(1, entryCount[0], "Entry action for ACTIVE should have fired");
  }

  @Test
  public void testEntryActionNotFiredOnIllegalTransition() {
    int[] entryCount = {0};
    machine.setEntryAction(TestState.SCORING, () -> entryCount[0]++);

    machine.requestTransition(TestState.SCORING); // IDLE→SCORING is illegal
    assertEquals(0, entryCount[0], "Entry action should not fire on rejected transition");
  }

  @Test
  public void testIsTransitionLegal() {
    assertTrue(machine.isTransitionLegal(TestState.ACTIVE));
    assertFalse(machine.isTransitionLegal(TestState.SCORING));
    assertTrue(machine.isTransitionLegal(TestState.EMERGENCY)); // wildcard-to
    assertTrue(machine.isTransitionLegal(TestState.IDLE)); // self-transition
  }

  @Test
  public void testOnTransitionCallback() {
    TestState[] captured = new TestState[2];
    machine.setOnTransition(
        (from, to) -> {
          captured[0] = from;
          captured[1] = to;
        });

    machine.requestTransition(TestState.ACTIVE);
    assertEquals(TestState.IDLE, captured[0]);
    assertEquals(TestState.ACTIVE, captured[1]);
  }
}
