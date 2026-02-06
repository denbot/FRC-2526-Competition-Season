import bot.den.foxflow.RobotState;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;

import java.util.Arrays;
import java.util.stream.Stream;

import static bot.den.frc2026.quaternion.helpers.TestHelpers.setDriverStationState;
import static edu.wpi.first.units.Units.Minutes;
import static edu.wpi.first.units.Units.Seconds;
import static org.junit.jupiter.api.Assertions.*;

public class MatchStateTest {
    @BeforeEach
    void setup() {
        assertTrue(HAL.initialize(500, 0));

        setDriverStationState(RobotState.DISABLED);

        assertFalse(SimHooks.isTimingPaused());
    }

    @AfterEach
    public void cleanup() {
        // This method runs after each test to reset the scheduler state
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().run(); // Call run() to execute end() methods
    }

    @Test
    void nothingHappensWhileRobotIsDisabled() {
        var machine = new RebuiltStateMachine();
        MatchState.setup(machine);

        machine.poll();
        assertEquals(RobotState.DISABLED, machine.currentState().robotState());
        assertEquals(MatchState.NONE, machine.currentState().matchState());

        // At least one full match later...
        SimHooks.stepTiming(Minutes.of(3).in(Seconds));

        machine.poll();

        assertEquals(RobotState.DISABLED, machine.currentState().robotState());
        assertEquals(MatchState.NONE, machine.currentState().matchState());
    }

    @Test
    void matchStateSwitchesToAutoWhenRobotSwitchesToAuto() {
        setDriverStationState(RobotState.AUTO);
        var machine = new RebuiltStateMachine();
        MatchState.setup(machine);

        machine.poll();

        // The first transition moves us into RobotState.AUTO
        assertEquals(RobotState.AUTO, machine.currentState().robotState());
        assertEquals(MatchState.NONE, machine.currentState().matchState());

        // The second transition moves the match state to Auto
        machine.poll();

        assertEquals(RobotState.AUTO, machine.currentState().robotState());
        assertEquals(MatchState.AUTO, machine.currentState().matchState());
    }

    @Test
    void matchStateGoesBackToNoneAfterTwentySeconds() {
        var machine = new RebuiltStateMachine(MatchState.AUTO);
        MatchState.setup(machine);

        SimHooks.stepTiming(MatchState.AUTO.timeInState.in(Seconds) - 1);
        machine.poll();
        assertEquals(MatchState.AUTO, machine.currentState().matchState());

        SimHooks.stepTiming(1);
        machine.poll();
        assertEquals(MatchState.NONE, machine.currentState().matchState());
    }

    @Test
    void matchStateMovesToTransitionShiftAfterTeleopStarts() {
        setDriverStationState(RobotState.TELEOP);
        var machine = new RebuiltStateMachine();
        MatchState.setup(machine);

        machine.poll();

        // The first transition moves us into RobotState.AUTO
        assertEquals(RobotState.TELEOP, machine.currentState().robotState());
        assertEquals(MatchState.NONE, machine.currentState().matchState());

        // The second transition moves the match state to Auto
        machine.poll();

        assertEquals(RobotState.TELEOP, machine.currentState().robotState());
        assertEquals(MatchState.TRANSITION_SHIFT, machine.currentState().matchState());
    }

    @Test
    void matchStateMovesFromTransitionShiftToShiftOne() {
        setDriverStationState(RobotState.TELEOP);
        var machine = new RebuiltStateMachine(MatchState.TRANSITION_SHIFT);
        MatchState.setup(machine);

        SimHooks.stepTiming(MatchState.TRANSITION_SHIFT.timeInState.in(Seconds) - 1);
        machine.poll();
        assertEquals(MatchState.TRANSITION_SHIFT, machine.currentState().matchState());

        SimHooks.stepTiming(1);
        machine.poll();
        assertEquals(MatchState.SHIFT_1, machine.currentState().matchState());
    }

    @Test
    void matchStateMovesFromShiftOneToShiftTwo() {
        setDriverStationState(RobotState.TELEOP);
        var machine = new RebuiltStateMachine(MatchState.SHIFT_1);
        MatchState.setup(machine);

        SimHooks.stepTiming(MatchState.SHIFT_1.timeInState.in(Seconds) - 1);
        machine.poll();
        assertEquals(MatchState.SHIFT_1, machine.currentState().matchState());

        SimHooks.stepTiming(1);
        machine.poll();
        assertEquals(MatchState.SHIFT_2, machine.currentState().matchState());
    }

    @Test
    void matchStateMovesFromShiftTwoToShiftThree() {
        setDriverStationState(RobotState.TELEOP);
        var machine = new RebuiltStateMachine(MatchState.SHIFT_2);
        MatchState.setup(machine);

        SimHooks.stepTiming(MatchState.SHIFT_2.timeInState.in(Seconds) - 1);
        machine.poll();
        assertEquals(MatchState.SHIFT_2, machine.currentState().matchState());

        SimHooks.stepTiming(1);
        machine.poll();
        assertEquals(MatchState.SHIFT_3, machine.currentState().matchState());
    }

    @Test
    void matchStateMovesFromShiftThreeToShiftFour() {
        setDriverStationState(RobotState.TELEOP);
        var machine = new RebuiltStateMachine(MatchState.SHIFT_3);
        MatchState.setup(machine);

        SimHooks.stepTiming(MatchState.SHIFT_3.timeInState.in(Seconds) - 1);
        machine.poll();
        assertEquals(MatchState.SHIFT_3, machine.currentState().matchState());

        SimHooks.stepTiming(1);
        machine.poll();
        assertEquals(MatchState.SHIFT_4, machine.currentState().matchState());
    }

    @Test
    void matchStateMovesFromShiftFourToEndGame() {
        setDriverStationState(RobotState.TELEOP);
        var machine = new RebuiltStateMachine(MatchState.SHIFT_4);
        MatchState.setup(machine);

        SimHooks.stepTiming(MatchState.SHIFT_4.timeInState.in(Seconds) - 1);
        machine.poll();
        assertEquals(MatchState.SHIFT_4, machine.currentState().matchState());

        SimHooks.stepTiming(1);
        machine.poll();
        assertEquals(MatchState.END_GAME, machine.currentState().matchState());
    }

    @Test
    void matchStateGoesBackToNoneAfterEndGame() {
        setDriverStationState(RobotState.TELEOP);
        var machine = new RebuiltStateMachine(MatchState.END_GAME);
        MatchState.setup(machine);

        SimHooks.stepTiming(MatchState.END_GAME.timeInState.in(Seconds) - 1);
        machine.poll();
        assertEquals(MatchState.END_GAME, machine.currentState().matchState());

        SimHooks.stepTiming(1);
        machine.poll();
        assertEquals(MatchState.NONE, machine.currentState().matchState());
    }

    @ParameterizedTest
    @MethodSource("matchAndRobotStateCombinations")
    void matchStateBecomesNoneAsSoonAsRobotIsDisabled(MatchState matchState, RobotState robotState) {
        if(matchState == MatchState.NONE) return;
        if(robotState == RobotState.DISABLED) return;

        var machine = new RebuiltStateMachine(matchState);
        MatchState.setup(machine);

        // Tell the driver station sim what state we're in
        setDriverStationState(robotState);
        machine.poll();

        assertEquals(robotState, machine.currentState().robotState());
        assertEquals(matchState, machine.currentState().matchState());

        // Now disable the robot
        setDriverStationState(RobotState.DISABLED);
        machine.poll();

        assertEquals(RobotState.DISABLED, machine.currentState().robotState());
        assertEquals(MatchState.NONE, machine.currentState().matchState());
    }

    private static Stream<Arguments> matchAndRobotStateCombinations() {
        return Arrays.stream(MatchState.values())
                .flatMap(a -> Arrays.stream(RobotState.values())
                        .map(b -> Arguments.arguments(a, b)));
    }
}
