package frc.robot.subsystems.intake;

import bot.den.foxflow.RobotState;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.state.HopperState;
import frc.robot.state.IntakeState;
import frc.robot.state.MatchState;
import frc.robot.state.RebuiltStateMachine;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.util.concurrent.atomic.AtomicBoolean;

import static frc.robot.helpers.TestHelpers.setDriverStationState;
import static org.junit.jupiter.api.Assertions.*;

public class IntakeTest {

    final AtomicBoolean leftTrigger = new AtomicBoolean();
    final AtomicBoolean xButton = new AtomicBoolean();
    private RebuiltStateMachine machine;
    private Intake intake;

    @BeforeEach
    void setup() {
        assertTrue(HAL.initialize(500, 0));

        setDriverStationState(RobotState.DISABLED);

        machine = new RebuiltStateMachine();
        IntakeIOSim intakeIOSim = new IntakeIOSim();
        intake = new Intake(intakeIOSim, machine);
    }

    @AfterEach
    public void cleanup() {
        // This method runs after each test to reset the scheduler state
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().run(); // Call run() to execute end() methods
    }

    @Test
    public void intakeIsDeployedAfterStateIsDeployingFromRetracted() {
        CommandScheduler.getInstance().schedule(machine.transitionTo(HopperState.RETRACTED));
        machine.poll();
        CommandScheduler.getInstance().schedule(machine.transitionTo(HopperState.DEPLOYING));
        machine.poll();
        intake.setIntakeMaxLength();
        SimHooks.stepTiming(100);
        assertEquals(HopperState.DEPLOYED, machine.currentState().hopperState());
    }

    @Test
    public void intakeIsDeployedAfterStateIsDeployingFromIdle() {
        CommandScheduler.getInstance().schedule(machine.transitionTo(HopperState.IDLE));
        machine.poll();
        CommandScheduler.getInstance().schedule(machine.transitionTo(HopperState.DEPLOYING));
        machine.poll();
        intake.setIntakeMaxLength();
        SimHooks.stepTiming(100);
        assertEquals(HopperState.DEPLOYED, machine.currentState().hopperState());
    }

    @Test
    public void intakeIsIdleAfterStateIsRetractingToIdle() {
        CommandScheduler.getInstance().schedule(machine.transitionTo(HopperState.DEPLOYED));
        machine.poll();
        CommandScheduler.getInstance().schedule(machine.transitionTo(HopperState.RETRACTING_TO_IDLE));
        machine.poll();
        intake.setIntakeIdleLength();
        SimHooks.stepTiming(100);
        assertEquals(HopperState.IDLE, machine.currentState().hopperState());
    }

    @Test
    public void intakeIsRetractedAfterStateIsRetractingToRetracted() {
        CommandScheduler.getInstance().schedule(machine.transitionTo(HopperState.DEPLOYED));
        machine.poll();
        CommandScheduler.getInstance().schedule(machine.transitionTo(HopperState.RETRACTING_TO_RETRACTED));
        machine.poll();
        intake.setIntakeMinLength();
        SimHooks.stepTiming(100);
        assertEquals(HopperState.RETRACTED, machine.currentState().hopperState());
    }
}
