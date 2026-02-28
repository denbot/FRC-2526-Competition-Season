package frc.robot.state;

import bot.den.foxflow.RobotState;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;

import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

import static frc.robot.helpers.TestHelpers.setDriverStationState;
import static org.junit.jupiter.api.Assertions.*;

public class ShooterStateTest {
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
        ShooterState.setup(machine, rightBumper::get, yButton::get);
    }

    @AfterEach
    public void cleanup() {
        // This method runs after each test to reset the scheduler state
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().run(); // Call run() to execute end() methods
    }

    @ParameterizedTest
    @MethodSource("buttonCombinations")
    void shooterSpinsUpWhenRightBumperOrYButtonHeld(boolean rightBumper, boolean yButton) {
        this.rightBumper.set(rightBumper);
        this.yButton.set(yButton);
        machine.poll();

        assertEquals(ShooterState.SPINNING_UP, machine.currentState().shooterState());
    }

    @ParameterizedTest
    @MethodSource("buttonCombinations")
    void shooterDoesntStopWhenButtonsNotReleased(boolean rightBumper, boolean yButton) {
        this.rightBumper.set(true);
        this.yButton.set(true);
        machine.poll();

        // Double-check spinner is spinning up
        assertEquals(ShooterState.SPINNING_UP, machine.currentState().shooterState());

        this.rightBumper.set(rightBumper);
        this.yButton.set(yButton);
        assertNotEquals(ShooterState.STOPPED, machine.currentState().shooterState());
    }

    @ParameterizedTest
    @MethodSource("buttonCombinations")
    void shooterStopsWhenAllButtonsReleased(boolean rightBumper, boolean yButton) {
        this.rightBumper.set(rightBumper);
        this.yButton.set(yButton);
        machine.poll();

        // Double-check spinner is spinning up
        assertEquals(ShooterState.SPINNING_UP, machine.currentState().shooterState());

        this.rightBumper.set(false);
        this.yButton.set(false);
        assertEquals(ShooterState.STOPPED, machine.currentState().shooterState());
    }

    private static List<Arguments> buttonCombinations() {
        return List.of(
                Arguments.arguments(true, false),
                Arguments.arguments(false, true),
                Arguments.arguments(true, true));
    }
}
