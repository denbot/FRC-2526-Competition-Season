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

public class HopperStateTest{

    final AtomicBoolean leftTrigger = new AtomicBoolean();
    final AtomicBoolean leftBumper = new AtomicBoolean();
    final AtomicBoolean bButton = new AtomicBoolean();
    private RebuiltStateMachine machine;

    @BeforeEach
    void setup() {
        assertTrue(HAL.initialize(500, 0));

        setDriverStationState(RobotState.DISABLED);

        machine = new RebuiltStateMachine();
        leftTrigger.set(false);
        leftBumper.set(false);
        bButton.set(false);
        HopperState.setup(machine, leftTrigger::get, leftBumper::get, bButton::get);
    }

    @AfterEach
    public void cleanup() {
        // This method runs after each test to reset the scheduler state
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().run(); // Call run() to execute end() methods
    }

    @Test
    public void hopperDeploysUsingLT() {
        leftTrigger.set(true);
        machine.poll();

        // Verify hopper begins to deploy
        assertEquals(HopperState.DEPLOYING, machine.currentState().hopperState());
    }

    @Test
    public void hopperDeploysUsingX() {
        bButton.set(true);
        machine.poll();

        // Verify hopper begins to deploy
        assertEquals(HopperState.DEPLOYING, machine.currentState().hopperState());
    }

    @Test
    public void hopperRetractsToIdleUsingLT() {
        leftTrigger.set(true);
        machine.poll();

        CommandScheduler.getInstance().schedule(machine.transitionTo(HopperState.DEPLOYED));

        leftTrigger.set(false);
        machine.poll();

        // Verify hopper begins to retract to idle
        assertEquals(HopperState.RETRACTING_TO_IDLE, machine.currentState().hopperState());
    }

    @Test
    public void hopperRetractsToIdleUsingX() {
        bButton.set(true);
        machine.poll();

        CommandScheduler.getInstance().schedule(machine.transitionTo(HopperState.DEPLOYED));

        bButton.set(false);
        machine.poll();

        // Verify hopper begins to retract to idle
        assertEquals(HopperState.RETRACTING_TO_IDLE, machine.currentState().hopperState());
    }

    @Test
    public void hopperCanDeployFromIdleUsingLT() {
        CommandScheduler.getInstance().schedule(machine.transitionTo(HopperState.IDLE));

        leftTrigger.set(true);
        machine.poll();

        // Verify hopper begins to deploy
        assertEquals(HopperState.DEPLOYING, machine.currentState().hopperState());
    }

    @Test
    public void hopperCanDeployFromIdleUsingX() {
        CommandScheduler.getInstance().schedule(machine.transitionTo(HopperState.IDLE));

        bButton.set(true);
        machine.poll();

        // Verify hopper begins to deploy
        assertEquals(HopperState.DEPLOYING, machine.currentState().hopperState());
    }

    @Test
    public void hopperCanFullyRetractFromDeployed() {
        leftTrigger.set(true);
        bButton.set(true);

        CommandScheduler.getInstance().schedule(machine.transitionTo(HopperState.DEPLOYED));

        leftBumper.set(true);
        machine.poll();

        // Verify hopper begins to retract to retracted
        assertEquals(HopperState.RETRACTING_TO_RETRACTED, machine.currentState().hopperState());
    }

    @Test
    public void hopperCanFullyRetractFromDeployedEvenWhenTryingToIdle() {
        CommandScheduler.getInstance().schedule(machine.transitionTo(HopperState.DEPLOYED));

        leftBumper.set(true);
        machine.poll();

        // Verify hopper begins to retract to retracted
        assertEquals(HopperState.RETRACTING_TO_RETRACTED, machine.currentState().hopperState());
    }

    @Test
    public void hopperCanFullyRetractFromIdle() {
        CommandScheduler.getInstance().schedule(machine.transitionTo(HopperState.IDLE));

        leftBumper.set(true);
        machine.poll();

        // Verify hopper begins to retract to retracted
        assertEquals(HopperState.RETRACTING_TO_RETRACTED, machine.currentState().hopperState());
    }
}
