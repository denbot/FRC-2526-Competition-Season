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
        button.set(true);
        retractingLimitSwitch.set(false);
        deployingLimitSwitch.set(false);
        HopperState.setup(machine, button::get, retractingLimitSwitch::get, deployingLimitSwitch::get);
        machine.poll();
        assertEquals(HopperState.DEPLOYING, machine.currentState().hopperState());
    }

    @Test
    public void hopperDeploysAfterLimitSwitch() {
        var machine = new RebuiltStateMachine();
        button.set(true);
        retractingLimitSwitch.set(false);
        deployingLimitSwitch.set(false);
        HopperState.setup(machine, button::get, retractingLimitSwitch::get, deployingLimitSwitch::get);
        machine.poll();
        button.set(false);
        deployingLimitSwitch.set(true);
        machine.poll();
        assertEquals(HopperState.DEPLOYED, machine.currentState().hopperState());
    }

    @Test
    public void rightBumperRetractsHopper() {
        var machine = new RebuiltStateMachine();
        button.set(true);
        retractingLimitSwitch.set(false);
        deployingLimitSwitch.set(false);
        HopperState.setup(machine, button::get, retractingLimitSwitch::get, deployingLimitSwitch::get);
        machine.poll();
        button.set(false);
        deployingLimitSwitch.set(true);
        machine.poll();
        button.set(true);
        HopperState.setup(machine, () -> true, () -> false, () -> true);
        machine.poll();
        assertEquals(HopperState.RETRACTING, machine.currentState().hopperState());
    }

    @Test
    public void hopperRetractsAfterLimitSwitch() {
        var machine = new RebuiltStateMachine();
        button.set(true);
        retractingLimitSwitch.set(false);
        deployingLimitSwitch.set(false);
        HopperState.setup(machine, button::get, retractingLimitSwitch::get, deployingLimitSwitch::get);
        machine.poll();
        button.set(false);
        deployingLimitSwitch.set(true);
        machine.poll();
        button.set(true);
        machine.poll();
        button.set(false);
        retractingLimitSwitch.set(true);
        deployingLimitSwitch.set(false);
        HopperState.setup(machine, () -> false, () -> true, () -> false);
        machine.poll();
        assertEquals(HopperState.RETRACTED, machine.currentState().hopperState());
    }


}
