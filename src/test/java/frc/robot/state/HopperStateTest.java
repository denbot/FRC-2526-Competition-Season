package frc.robot.state;

import bot.den.foxflow.RobotState;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOSim;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicLong;

import static frc.robot.helpers.TestHelpers.setDriverStationState;
import static org.junit.jupiter.api.Assertions.*;

public class HopperStateTest{

    final AtomicBoolean leftTrigger = new AtomicBoolean();
    final AtomicBoolean leftBumper = new AtomicBoolean();
    final AtomicBoolean xButton = new AtomicBoolean();
    final AtomicLong closedLoopError = new AtomicLong();
    private RebuiltStateMachine machine;

    @BeforeEach
    void setup() {
        assertTrue(HAL.initialize(500, 0));

        setDriverStationState(RobotState.DISABLED);

        machine = new RebuiltStateMachine();
        leftTrigger.set(false);
        leftBumper.set(false);
        closedLoopError.set(0);
        HopperState.setup(machine, leftTrigger::get, leftBumper::get, xButton::get);

        Intake intake = new Intake(new IntakeIOSim());
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
        closedLoopError.set(999);
        machine.poll();

        // Verify hopper begins to deploy
        assertEquals(HopperState.DEPLOYING, machine.currentState().hopperState());

        closedLoopError.set(0);
        machine.poll();

        // Verify hopper gets deployed
        assertEquals(HopperState.DEPLOYED, machine.currentState().hopperState());
    }

    @Test
    public void hopperRetractsToIdle() {
        leftTrigger.set(true);
        closedLoopError.set(0);
        machine.poll();

        // Verify hopper gets deployed
        assertEquals(HopperState.DEPLOYED, machine.currentState().hopperState());

        leftTrigger.set(false);
        closedLoopError.set(999);
        machine.poll();

        // Verify hopper begins to retract to idle
        assertEquals(HopperState.RETRACTING_TO_IDLE, machine.currentState().hopperState());

        closedLoopError.set(0);
        machine.poll();

        // Verify hopper retracts to idle
        assertEquals(HopperState.IDLE, machine.currentState().hopperState());
    }

    @Test
    public void hopperCanDeployFromIdle() {
        leftTrigger.set(true);
        closedLoopError.set(0);
        machine.poll();

        // Double check that hopper gets deployed
        assertEquals(HopperState.DEPLOYED, machine.currentState().hopperState());

        leftTrigger.set(false);
        closedLoopError.set(0);
        machine.poll();

        // Double check hopper retracts to idle
        assertEquals(HopperState.IDLE, machine.currentState().hopperState());

        leftTrigger.set(true);
        closedLoopError.set(999);
        machine.poll();

        // Verify hopper begins to deploy
        assertEquals(HopperState.DEPLOYING, machine.currentState().hopperState());

        closedLoopError.set(0);
        machine.poll();

        // Verify hopper gets deployed
        assertEquals(HopperState.DEPLOYED, machine.currentState().hopperState());
    }

    @Test
    public void hopperCanFullyRetractFromIdle() {
        leftTrigger.set(true);
        closedLoopError.set(0);
        machine.poll();

        // Double check that hopper gets deployed
        assertEquals(HopperState.DEPLOYED, machine.currentState().hopperState());

        leftTrigger.set(false);
        closedLoopError.set(0);
        machine.poll();

        // Double check hopper retracts to idle
        assertEquals(HopperState.IDLE, machine.currentState().hopperState());

        leftBumper.set(true);
        closedLoopError.set(999);
        machine.poll();

        // Verify that hopper begins retracting to fully retracted
        assertEquals(HopperState.RETRACTING_TO_RETRACTED, machine.currentState().hopperState());

        closedLoopError.set(0);
        machine.poll();

        // Verify that hopper begins retracting to fully retracted
        assertEquals(HopperState.RETRACTED, machine.currentState().hopperState());
    }

}
