package frc.robot.state;

import bot.den.foxflow.RobotState;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.util.concurrent.atomic.AtomicBoolean;

import static frc.robot.helpers.TestHelpers.setDriverStationState;
import static org.junit.jupiter.api.Assertions.*;

public class KickerStateTest {

    final AtomicBoolean rightTrigger = new AtomicBoolean();
    final AtomicBoolean leftTrigger = new AtomicBoolean();
    final AtomicBoolean xButton = new AtomicBoolean();
    private RebuiltStateMachine machine;

    @BeforeEach
    void setup() {
        assertTrue(HAL.initialize(500, 0));

        setDriverStationState(RobotState.DISABLED);

        machine = new RebuiltStateMachine();
        leftTrigger.set(false);
        xButton.set(false);
        KickerState.setup(machine, rightTrigger::get, leftTrigger::get, xButton::get);
    }

    @AfterEach
    public void cleanup() {
        // This method runs after each test to reset the scheduler state
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().run(); // Call run() to execute end() methods
    }

    @Test
    void rightTriggerCausesKickerToRun() { // TODO: Check if the flywheels are up to speed
        rightTrigger.set(true);
        machine.poll();

        assertEquals(KickerState.RUNNING, machine.currentState().kickerState());
    }

    @Test
    void kickerStopsIfRightTriggerReleased() { // TODO: Check if the flywheels are up to speed
        rightTrigger.set(true);
        machine.poll();

        // Double check that it's running
        assertEquals(KickerState.RUNNING, machine.currentState().kickerState());

        // Release the trigger
        rightTrigger.set(false);
        machine.poll();

        assertEquals(KickerState.STOPPED, machine.currentState().kickerState());
    }

    @Test
    void leftTriggerCausesKickerToReverse() {
        leftTrigger.set(true);
        machine.poll();

        assertEquals(KickerState.REVERSING, machine.currentState().kickerState());
    }

    @Test
    void kickerStopsIfLeftTriggerReleased() {
        leftTrigger.set(true);
        machine.poll();

        // Double check that it's reversing
        assertEquals(KickerState.REVERSING, machine.currentState().kickerState());

        // Release the trigger
        leftTrigger.set(false);
        machine.poll();

        assertEquals(KickerState.STOPPED, machine.currentState().kickerState());
    }

    @Test
    void xButtonCausesKickerToReverse() {
        xButton.set(true);
        machine.poll();

        assertEquals(KickerState.REVERSING, machine.currentState().kickerState());
    }

    @Test
    void kickerStopsIfXButtonReleased() {
        xButton.set(true);
        machine.poll();

        // Double check that it's reversing
        assertEquals(KickerState.REVERSING, machine.currentState().kickerState());

        // Release the trigger
        xButton.set(false);
        machine.poll();

        assertEquals(KickerState.STOPPED, machine.currentState().kickerState());
    }
}
