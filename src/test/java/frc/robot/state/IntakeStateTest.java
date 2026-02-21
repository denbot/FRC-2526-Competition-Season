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

public class IntakeStateTest {

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
        IntakeState.setup(machine, leftTrigger::get, xButton::get);
    }

    @AfterEach
    public void cleanup() {
        // This method runs after each test to reset the scheduler state
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().run(); // Call run() to execute end() methods
    }

    @Test
    void intakeBecomesActiveIfTriggerPressed() {
        leftTrigger.set(true);
        machine.poll();

        assertEquals(IntakeState.RUNNING, machine.currentState().intakeState());
    }

    @Test
    void intakeStopsIfTriggerReleased() {
        leftTrigger.set(true);
        machine.poll();

        // Double check that it's running
        assertEquals(IntakeState.RUNNING, machine.currentState().intakeState());

        // Release the trigger
        leftTrigger.set(false);
        machine.poll();

        assertEquals(IntakeState.STOPPED, machine.currentState().intakeState());
    }

    @Test
    void intakeReversesWhenXButtonIsPressed() {
        xButton.set(true);
        machine.poll();

        assertEquals(IntakeState.REVERSING, machine.currentState().intakeState());
    }

    @Test
    void intakeStopsWhenXButtonIsReleased() {
        xButton.set(true);
        machine.poll();

        // Double check that it's reversing
        assertEquals(IntakeState.REVERSING, machine.currentState().intakeState());

        // Release the button
        xButton.set(false);
        machine.poll();

        assertEquals(IntakeState.STOPPED, machine.currentState().intakeState());
    }
}
