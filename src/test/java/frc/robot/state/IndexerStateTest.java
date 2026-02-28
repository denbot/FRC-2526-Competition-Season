package frc.robot.state;

import bot.den.foxflow.RobotState;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.indexer.Indexer;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.util.concurrent.atomic.AtomicBoolean;

import static frc.robot.helpers.TestHelpers.setDriverStationState;
import static org.junit.jupiter.api.Assertions.*;

public class IndexerStateTest {

    final AtomicBoolean rightTrigger = new AtomicBoolean();
    final AtomicBoolean leftTrigger = new AtomicBoolean();
    final AtomicBoolean aButton = new AtomicBoolean();
    final AtomicBoolean xButton = new AtomicBoolean();
    private RebuiltStateMachine machine;

    @BeforeEach
    void setup() {
        assertTrue(HAL.initialize(500, 0));

        setDriverStationState(RobotState.DISABLED);

        machine = new RebuiltStateMachine();
        rightTrigger.set(false);
        leftTrigger.set(false);
        aButton.set(false);
        xButton.set(false);
        IndexerState.setup(machine, rightTrigger::get, leftTrigger::get, aButton::get, xButton::get);
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
    void indexerBecomesActiveIfRightTriggerPressed() {
        rightTrigger.set(true);
        machine.poll();

        assertEquals(IndexerState.RUNNING, machine.currentState().indexerState());
    }
    @Test
    void indexerStopsIfRightTriggerReleased() {
        rightTrigger.set(true);
        machine.poll();

        // Double check that it's running
        assertEquals(IndexerState.RUNNING, machine.currentState().indexerState());

        // Release the trigger

        rightTrigger.set(false);
        machine.poll();

        assertEquals(IndexerState.STOPPED, machine.currentState().indexerState());
    }
    @Test
    void indexerReversesIfAButtonPressed() {
        aButton.set(true);
        machine.poll();

        assertEquals(IndexerState.REVERSING, machine.currentState().indexerState());
    }

    @Test
    void indexerStopsIfAButtonReleased() {
        aButton.set(true);
        machine.poll();

        // Double check that it's running
        assertEquals(IndexerState.REVERSING, machine.currentState().indexerState());

        // Release the trigger

        aButton.set(false);
        machine.poll();

        assertEquals(IndexerState.STOPPED, machine.currentState().indexerState());
    }

    @Test
    void indexerReversesIfXButtonPressed() {
        xButton.set(true);
        machine.poll();

        assertEquals(IndexerState.REVERSING, machine.currentState().indexerState());
    }

    @Test
    void indexerStopsIfXButtonReleased() {
        xButton.set(true);
        machine.poll();

        // Double check that it's running
        assertEquals(IndexerState.REVERSING, machine.currentState().indexerState());

        // Release the trigger

        xButton.set(false);
        machine.poll();

        assertEquals(IndexerState.STOPPED, machine.currentState().indexerState());
    }

    @Test
    void indexerReversesIfXAndAButtonPressed() {
        xButton.set(true);
        aButton.set(true);
        machine.poll();

        assertEquals(IndexerState.REVERSING, machine.currentState().indexerState());
    }

    @Test
    void indexerStopsIfXAndAButtonReleased() {
        xButton.set(true);
        aButton.set(true);
        machine.poll();

        // Double check that it's running
        assertEquals(IndexerState.REVERSING, machine.currentState().indexerState());

        // Release the trigger

        xButton.set(false);
        aButton.set(false);
        machine.poll();

        assertEquals(IndexerState.STOPPED, machine.currentState().indexerState());
    }

    @Test
    void indexerBecomesActiveIfLeftAndRightTriggerPressed() {
        leftTrigger.set(true);
        rightTrigger.set(true);
        machine.poll();

        assertEquals(IndexerState.RUNNING, machine.currentState().indexerState());
    }

    @Test
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
}