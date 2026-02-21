package frc.robot.state;

import bot.den.foxflow.RobotState;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import static frc.robot.helpers.TestHelpers.setDriverStationState;
import static org.junit.jupiter.api.Assertions.*;

public class HopperStateTest{
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
    public void rightBumperDeploysHopper() {
        var machine = new RebuiltStateMachine();
        HopperState.setup(machine, () -> true, () -> false, () -> false);
        machine.poll();
        assertEquals(HopperState.DEPLOYING, machine.currentState().hopperState());
    }

    @Test
    public void hopperDeploysAfterLimitSwitch() {
        var machine = new RebuiltStateMachine();
        HopperState.setup(machine, () -> true, () -> false, () -> false);
        machine.poll();
        HopperState.setup(machine, () -> false, () -> false, () -> true);
        machine.poll();
        assertEquals(HopperState.DEPLOYED, machine.currentState().hopperState());
    }

    @Test
    public void rightBumperRetractsHopper() {
        var machine = new RebuiltStateMachine();
        HopperState.setup(machine, () -> true, () -> false, () -> false);
        machine.poll();
        HopperState.setup(machine, () -> false, () -> false, () -> true);
        machine.poll();
        HopperState.setup(machine, () -> true, () -> false, () -> true);
        machine.poll();
        assertEquals(HopperState.RETRACTING, machine.currentState().hopperState());
    }

    @Test
    public void hopperRetractsAfterLimitSwitch() {
        var machine = new RebuiltStateMachine();
        HopperState.setup(machine, () -> true, () -> false, () -> false);
        machine.poll();
        HopperState.setup(machine, () -> false, () -> false, () -> true);
        machine.poll();
        HopperState.setup(machine, () -> true, () -> false, () -> true);
        machine.poll();
        HopperState.setup(machine, () -> false, () -> true, () -> false);
        machine.poll();
        assertEquals(HopperState.RETRACTED, machine.currentState().hopperState());
    }


}
