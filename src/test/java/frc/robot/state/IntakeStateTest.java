package frc.robot.state;

import bot.den.foxflow.RobotState;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import static frc.robot.helpers.TestHelpers.setDriverStationState;
import static org.junit.jupiter.api.Assertions.*;

public class IntakeStateTest {
    @BeforeEach
    void setup() {
        assertTrue(HAL.initialize(500, 0));

        setDriverStationState(RobotState.DISABLED);
    }

    @AfterEach
    public void cleanup() {
        // This method runs after each test to reset the scheduler state
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().run(); // Call run() to execute end() methods
    }

    @Test
    void intakeBecomesActiveIfButtonPressedAndHopperDeployed() {
        var machine = new RebuiltStateMachine();
        IntakeState.setup(machine, () -> true, () -> false);
        CommandScheduler.getInstance().schedule(machine.transitionTo(HopperState.DEPLOYED));
        machine.poll();
        assertEquals(IntakeState.ACTIVE, machine.currentState().intakeState());
    }

    @Test
    void intakeBecomesInactiveIfButtonReleased() {
        var machine = new RebuiltStateMachine();
        IntakeState.setup(machine, () -> true, () -> false);
        CommandScheduler.getInstance().schedule(machine.transitionTo(HopperState.DEPLOYED));
        machine.poll();
        IntakeState.setup(machine, () -> false, () -> false);
        machine.poll();
        assertEquals(IntakeState.INACTIVE, machine.currentState().intakeState());
    }

    @Test
    void intakeBecomesInactiveIfHopperRetracts() {
        var machine = new RebuiltStateMachine();
        IntakeState.setup(machine, () -> true, () -> false);
        CommandScheduler.getInstance().schedule(machine.transitionTo(HopperState.DEPLOYED));
        CommandScheduler.getInstance().schedule(machine.transitionTo(HopperState.RETRACTING));
        machine.poll();
        assertEquals(IntakeState.INACTIVE, machine.currentState().intakeState());
    }

    @Test
    void intakeReversesWhenYButtonIsPressed() {
        var machine = new RebuiltStateMachine();
        IntakeState.setup(machine, () -> false, () -> true);
        machine.poll();
        assertEquals(IntakeState.REVERSE, machine.currentState().intakeState());
    }

    @Test
    void intakeStopsReversingWhenYButtonIsReleased() {
        var machine = new RebuiltStateMachine();
        IntakeState.setup(machine, () -> false, () -> true);
        machine.poll();
        IntakeState.setup(machine, () -> false, () -> false);
        machine.poll();
        assertEquals(IntakeState.INACTIVE, machine.currentState().intakeState());
    }


}
