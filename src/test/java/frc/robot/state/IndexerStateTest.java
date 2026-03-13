package frc.robot.state;

import bot.den.foxflow.RobotState;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.indexer.Indexer;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;

import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

import static frc.robot.helpers.TestHelpers.setDriverStationState;
import static org.junit.jupiter.api.Assertions.*;

public class IndexerStateTest {

    final AtomicBoolean rightTrigger = new AtomicBoolean();
    final AtomicBoolean leftTrigger = new AtomicBoolean();
    final AtomicBoolean aButton = new AtomicBoolean();
    final AtomicBoolean bButton = new AtomicBoolean();
    private RebuiltStateMachine machine;

    @BeforeEach
    void setup() {
        assertTrue(HAL.initialize(500, 0));

        setDriverStationState(RobotState.DISABLED);

        machine = new RebuiltStateMachine();
        rightTrigger.set(false);
        leftTrigger.set(false);
        aButton.set(false);
        bButton.set(false);
        IndexerState.setup(machine, rightTrigger::get, leftTrigger::get, aButton::get, bButton::get);
    }

    @AfterEach
    public void cleanup() {
        // This method runs after each test to reset the scheduler state
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().run(); // Call run() to execute end() methods
    }

    @Test
    void indexerBecomesActiveIfLeftTriggerPressed() {
        leftTrigger.set(true);
        machine.poll();

        assertEquals(IndexerState.RUNNING, machine.currentState().indexerState());
    }

    @Test
    void indexerStopsIfLeftTriggerReleased() {
        leftTrigger.set(true);
        machine.poll();

        // Double check that it's running
        assertEquals(IndexerState.RUNNING, machine.currentState().indexerState());

        // Release the trigger

        leftTrigger.set(false);
        machine.poll();

        assertEquals(IndexerState.STOPPED, machine.currentState().indexerState());
    }

    @Test
    void indexerBecomesActiveIfRightTriggerPressedAndShooterStateActive() {
        CommandScheduler.getInstance().schedule(machine.transitionTo(ShooterState.AT_SPEED));

        rightTrigger.set(true);
        machine.poll();

        assertEquals(IndexerState.RUNNING, machine.currentState().indexerState());
    }

    @Test
    void indexerStopsIfRightTriggerReleased() {
        CommandScheduler.getInstance().schedule(machine.transitionTo(ShooterState.AT_SPEED));

        rightTrigger.set(true);
        machine.poll();

        // Double check that it's running
        assertEquals(IndexerState.RUNNING, machine.currentState().indexerState());

        // Release the trigger

        rightTrigger.set(false);
        machine.poll();

        assertEquals(IndexerState.STOPPED, machine.currentState().indexerState());
    }

    @Test // Isnt be needed but is a just in case
    void indexerBecomesActiveIfLeftAndRightTriggerPressed() {
        leftTrigger.set(true);
        rightTrigger.set(true);
        machine.poll();

        assertEquals(IndexerState.RUNNING, machine.currentState().indexerState());
    }

    @Test // Isnt be needed but is a just in case
    void indexerStopsIfLeftAndRightTriggerReleased() {
        rightTrigger.set(true);
        leftTrigger.set(true);
        machine.poll();

        // Double check that it's running
        assertEquals(IndexerState.RUNNING, machine.currentState().indexerState());

        // Release the trigger

        rightTrigger.set(false);
        leftTrigger.set(false);
        machine.poll();

        assertEquals(IndexerState.STOPPED, machine.currentState().indexerState());
    }

    @ParameterizedTest
    @MethodSource("notAtSpeedShooterStates")
    void rightTriggerWontMakeIndexerActiveIfNotAtSpeed(ShooterState shooterState) {
        CommandScheduler.getInstance().schedule(machine.transitionTo(shooterState));

        rightTrigger.set(true);
        machine.poll();

        assertEquals(IndexerState.STOPPED, machine.currentState().indexerState());
    }

    @ParameterizedTest
    @MethodSource("buttonCombinations")
    void indexerReversesIfAOrbButtonReleased(boolean a, boolean x) {
        aButton.set(a);
        bButton.set(x);
        machine.poll();

        assertEquals(IndexerState.REVERSING, machine.currentState().indexerState());
    }

    @ParameterizedTest
    @MethodSource("buttonCombinations")
    void indexerStopsReversingIfAllButtonsReleased(boolean a, boolean x) {
        aButton.set(a);
        bButton.set(x);
        machine.poll();

        // Double check that it's running
        assertEquals(IndexerState.REVERSING, machine.currentState().indexerState());

        // Release the buttons

        aButton.set(false);
        bButton.set(false);
        machine.poll();

        assertEquals(IndexerState.STOPPED, machine.currentState().indexerState());
    }

    private static List<Arguments> notAtSpeedShooterStates() {
        return List.of(
                Arguments.arguments(ShooterState.STOPPED),
                Arguments.arguments(ShooterState.SPINNING_UP_ADAPTIVE),
                Arguments.arguments(ShooterState.SPINNING_UP_FIXED));
    }
    
    private static List<Arguments> buttonCombinations() {
        return List.of(
                Arguments.arguments(true, false),
                Arguments.arguments(false, true),
                Arguments.arguments(true, true));
    }
}