package frc.robot.subsystems.shooter;

import bot.den.foxflow.RobotState;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.generated.TunerConstants;
import frc.robot.state.HopperState;
import frc.robot.state.RebuiltStateMachine;
import frc.robot.state.ShooterState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import static frc.robot.helpers.TestHelpers.setDriverStationState;
import static org.junit.jupiter.api.Assertions.*;

public class ShooterTest {

    private RebuiltStateMachine machine;
    private Shooter shooter;

    @BeforeEach
    void setup() {
        assertTrue(HAL.initialize(500, 0));

        setDriverStationState(RobotState.DISABLED);

        machine = new RebuiltStateMachine();
        ShooterIOSim shooterIOSim = new ShooterIOSim();
        Drive drive =
                new Drive(
                        new GyroIO() {},
                        new ModuleIOSim(TunerConstants.FrontLeft),
                        new ModuleIOSim(TunerConstants.FrontRight),
                        new ModuleIOSim(TunerConstants.BackLeft),
                        new ModuleIOSim(TunerConstants.BackRight));
        shooter = new Shooter(shooterIOSim, machine, drive);
    }

    @AfterEach
    public void cleanup() {
        // This method runs after each test to reset the scheduler state
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().run(); // Call run() to execute end() methods
    }

    @Test
    void shooterSpinsUpAndChangesStateAdaptive() {
        CommandScheduler.getInstance().schedule(machine.transitionTo(ShooterState.SPINNING_UP_ADAPTIVE));
        machine.poll();
        SimHooks.stepTiming(100);
        machine.poll();
        assertEquals(ShooterState.AT_SPEED, machine.currentState().shooterState());
    }

    @Test
    void shooterSpinsUpAndChangesStateFixed() {
        CommandScheduler.getInstance().schedule(machine.transitionTo(ShooterState.SPINNING_UP_FIXED));
        machine.poll();
        SimHooks.stepTiming(100);
        machine.poll();
        assertEquals(ShooterState.AT_SPEED, machine.currentState().shooterState());
    }
}
