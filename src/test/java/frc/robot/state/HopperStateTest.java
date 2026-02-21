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

    final AtomicBoolean button = new AtomicBoolean();
    final AtomicBoolean retractingLimitSwitch = new AtomicBoolean();
    final AtomicBoolean deployingLimitSwitch = new AtomicBoolean();
    private RebuiltStateMachine machine;

    @BeforeEach
    void setup() {
        assertTrue(HAL.initialize(500, 0));

        setDriverStationState(RobotState.DISABLED);

        machine = new RebuiltStateMachine();
        button.set(false);
        retractingLimitSwitch.set(false);
        deployingLimitSwitch.set(false);
        HopperState.setup(machine, button::get, retractingLimitSwitch::get, deployingLimitSwitch::get);
    }

    @AfterEach
    public void cleanup() {
        // This method runs after each test to reset the scheduler state
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().run(); // Call run() to execute end() methods
    }

    @Test
    public void hopperWorksAsExpected() {
        button.set(true);
        machine.poll();

        // Verify hopper begins to deploy
        assertEquals(HopperState.DEPLOYING, machine.currentState().hopperState());

        button.set(false);
        deployingLimitSwitch.set(true);
        machine.poll();

        // Verify hopper gets deployed
        assertEquals(HopperState.DEPLOYED, machine.currentState().hopperState());

        button.set(true);
        machine.poll();

        // Verify hopper begins to retract
        assertEquals(HopperState.RETRACTING, machine.currentState().hopperState());

        button.set(false);
        retractingLimitSwitch.set(true);
        deployingLimitSwitch.set(false);
        machine.poll();

        // Verify hopper gets retracted
        assertEquals(HopperState.RETRACTED, machine.currentState().hopperState());
    }
}
