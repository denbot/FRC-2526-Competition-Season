package frc.robot.state;

import bot.den.foxflow.RobotState;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOSim;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.util.concurrent.atomic.AtomicBoolean;

import static frc.robot.helpers.TestHelpers.setDriverStationState;
import static org.junit.jupiter.api.Assertions.*;

public class HopperStateTest{

    final AtomicBoolean leftTrigger = new AtomicBoolean();
    final AtomicBoolean leftBumper = new AtomicBoolean();
    final AtomicBoolean xButton = new AtomicBoolean();
    private RebuiltStateMachine machine;

    @BeforeEach
    void setup() {
        assertTrue(HAL.initialize(500, 0));

        setDriverStationState(RobotState.DISABLED);

        machine = new RebuiltStateMachine();
        leftTrigger.set(false);
        leftBumper.set(false);
        HopperState.setup(machine, leftTrigger::get, leftBumper::get, xButton::get);
    }

    @AfterEach
    public void cleanup() {
        // This method runs after each test to reset the scheduler state
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().run(); // Call run() to execute end() methods
    }

    @Test
    public void hopperDeploys() {
        leftTrigger.set(true);
        machine.poll();

        // Verify hopper begins to deploy
        assertEquals(HopperState.DEPLOYING, machine.currentState().hopperState());
    }

    @Test
    public void hopperRetractsToIdle() {
        leftTrigger.set(true);
        machine.poll();

        CommandScheduler.getInstance().schedule(machine.transitionTo(HopperState.DEPLOYED));

        leftTrigger.set(false);
        machine.poll();

        // Verify hopper begins to retract to idle
        assertEquals(HopperState.RETRACTING_TO_IDLE, machine.currentState().hopperState());
    }

    @Test
    public void hopperCanDeployFromIdle() {
        CommandScheduler.getInstance().schedule(machine.transitionTo(HopperState.IDLE));

        leftTrigger.set(true);
        machine.poll();

        // Verify hopper begins to retract to idle
        assertEquals(HopperState.DEPLOYING, machine.currentState().hopperState());
    }

    @Test
    public void hopperCanFullyRetractFromIdle() {
        CommandScheduler.getInstance().schedule(machine.transitionTo(HopperState.IDLE));

        leftBumper.set(true);
        machine.poll();

        // Verify hopper begins to retract to idle
        assertEquals(HopperState.RETRACTING_TO_RETRACTED, machine.currentState().hopperState());
    }
}
