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

    final AtomicBoolean rightBumper = new AtomicBoolean();
    final AtomicBoolean yButton = new AtomicBoolean();
    private RebuiltStateMachine machine;

    @BeforeEach
    void setup() {
        assertTrue(HAL.initialize(500, 0));

        setDriverStationState(RobotState.DISABLED);

        machine = new RebuiltStateMachine();
        rightBumper.set(false);
        yButton.set(false);
        IntakeState.setup(machine, rightBumper::get, yButton::get);
    }

    @AfterEach
    public void cleanup() {
        // This method runs after each test to reset the scheduler state
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().run(); // Call run() to execute end() methods
    }

    @Test
    void intakeBecomesActiveIfButtonPressedAndHopperDeployed() {
        CommandScheduler.getInstance().schedule(machine.transitionTo(HopperState.DEPLOYED));

        machine.poll();

        // Verify Intake isn't active yet
        assertEquals(IntakeState.INACTIVE, machine.currentState().intakeState());

        rightBumper.set(true);
        machine.poll();

        // Verify both are required
        assertEquals(IntakeState.ACTIVE, machine.currentState().intakeState());
    }

    @Test
    void intakeBecomesInactiveIfButtonReleased() {
        CommandScheduler.getInstance().schedule(machine.transitionTo(HopperState.DEPLOYED));
        rightBumper.set(true);

        // Double check that it's active
        machine.poll();
        assertEquals(IntakeState.ACTIVE, machine.currentState().intakeState());

        // Release the button
        rightBumper.set(false);
        machine.poll();

        assertEquals(IntakeState.INACTIVE, machine.currentState().intakeState());
    }

    @Test
    void intakeBecomesInactiveIfHopperRetracts() {
        CommandScheduler.getInstance().schedule(machine.transitionTo(HopperState.DEPLOYED));
        rightBumper.set(true);
        machine.poll();

        //  Double check that it's active
        assertEquals(IntakeState.ACTIVE, machine.currentState().intakeState());

        // Retract the hopper
        //CommandScheduler.getInstance().schedule(machine.transitionTo(HopperState.RETRACTING));
        machine.poll();

        assertEquals(IntakeState.INACTIVE, machine.currentState().intakeState());
    }

    @Test
    void intakeReversesWhenYButtonIsPressed() {
        yButton.set(true);
        machine.poll();
        assertEquals(IntakeState.REVERSE, machine.currentState().intakeState());
    }

    @Test
    void intakeStopsReversingWhenYButtonIsReleased() {
        yButton.set(true);
        machine.poll();

        assertEquals(IntakeState.REVERSE, machine.currentState().intakeState());

        yButton.set(false);
        machine.poll();

        assertEquals(IntakeState.INACTIVE, machine.currentState().intakeState());
    }

}
