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

public class KickerStateTest {

    final AtomicBoolean rightTrigger = new AtomicBoolean();
    final AtomicBoolean leftTrigger = new AtomicBoolean();
    final AtomicBoolean bButton = new AtomicBoolean();
    final AtomicBoolean aButton = new AtomicBoolean();
    final AtomicBoolean churnTrigger = new AtomicBoolean();
    private RebuiltStateMachine machine;

    @BeforeEach
    void setup() {
        assertTrue(HAL.initialize(500, 0));

        setDriverStationState(RobotState.DISABLED);

        machine = new RebuiltStateMachine();
        bButton.set(false);
        KickerState.setup(machine, rightTrigger::get, leftTrigger::get, bButton::get, aButton::get, churnTrigger::get);
    }

    @AfterEach
    public void cleanup() {
        // This method runs after each test to reset the scheduler state
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().run(); // Call run() to execute end() methods
    }

    @Test
    void rightTriggerCausesKickerToRunIfAtSpeed() {
        CommandScheduler.getInstance().schedule(machine.transitionTo(ShooterState.AT_SPEED));

        rightTrigger.set(true);
        machine.poll();

        assertEquals(KickerState.RUNNING, machine.currentState().kickerState());
    }

    @ParameterizedTest
    @MethodSource("notAtSpeedShooterStates")
    void rightTriggerDoesntCauseKickerToRunIfNotAtSpeed(ShooterState state) {
        CommandScheduler.getInstance().schedule(machine.transitionTo(state));

        rightTrigger.set(true);
        machine.poll();

        assertNotEquals(KickerState.RUNNING, machine.currentState().kickerState());
    }

    @Test
    void kickerStopsIfRightTriggerReleased() {
        CommandScheduler.getInstance().schedule(machine.transitionTo(ShooterState.AT_SPEED));

        rightTrigger.set(true);
        machine.poll();

        // Double check that it's running
        assertEquals(KickerState.RUNNING, machine.currentState().kickerState());

        // Release the trigger
        rightTrigger.set(false);
        machine.poll();

        assertEquals(KickerState.STOPPED, machine.currentState().kickerState());
    }

    @ParameterizedTest
    @MethodSource("notAtSpeedShooterStates")
    void kickerStopsIfShooterStops(ShooterState state) {
        CommandScheduler.getInstance().schedule(machine.transitionTo(ShooterState.AT_SPEED));

        rightTrigger.set(true);
        machine.poll();

        // Double check that it's running
        assertEquals(KickerState.RUNNING, machine.currentState().kickerState());

        CommandScheduler.getInstance().schedule(machine.transitionTo(state));
        machine.poll();

        assertNotEquals(KickerState.RUNNING, machine.currentState().kickerState());
    }

    @Test
    void kickerReversesWithBButton() {
        bButton.set(true);
        machine.poll();

        assertEquals(KickerState.REVERSING, machine.currentState().kickerState());
    }

    @Test
    void kickerStopsWhenBButtonReleased() {
        bButton.set(true);
        machine.poll();

        // Double check that it's reversing
        assertEquals(KickerState.REVERSING, machine.currentState().kickerState());

        // Release the button
        bButton.set(false);
        machine.poll();

        assertEquals(KickerState.STOPPED, machine.currentState().kickerState());
    }

    private static List<Arguments> notAtSpeedShooterStates() {
        return List.of(
                Arguments.arguments(ShooterState.STOPPED),
                Arguments.arguments(ShooterState.SPINNING_UP_ADAPTIVE),
                Arguments.arguments(ShooterState.SPINNING_UP_FIXED));
    }
}
