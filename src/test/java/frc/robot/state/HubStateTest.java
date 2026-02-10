package frc.robot.state;

import bot.den.foxflow.RobotState;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.helpers.TestHelpers;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.EnumSource;
import org.junit.jupiter.params.provider.MethodSource;

import java.util.Arrays;
import java.util.stream.Stream;

import static frc.robot.helpers.TestHelpers.setDriverStationState;
import static org.junit.jupiter.api.Assertions.*;

public class HubStateTest {
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
    void whenTheMatchStateIsAutoTheHubIsActive() {
        var machine = new RebuiltStateMachine();
        HubState.setup(machine);

        CommandScheduler.getInstance().schedule(machine.transitionTo(MatchState.AUTO));

        assertEquals(MatchState.AUTO, machine.currentState().matchState());
        assertEquals(HubState.ACTIVE, machine.currentState().hubState());
    }

    @ParameterizedTest
    @EnumSource(MatchState.class)
    void whenTheMatchStateGoesBackToNoneTheHubIsInactive(MatchState startingState) {
        if(startingState == MatchState.NONE) return;

        var machine = new RebuiltStateMachine(startingState, HubState.ACTIVE);
        HubState.setup(machine);

        CommandScheduler.getInstance().schedule(machine.transitionTo(MatchState.NONE));

        assertEquals(MatchState.NONE, machine.currentState().matchState());
        assertEquals(HubState.INACTIVE, machine.currentState().hubState());
    }

    @Test
    void whenTheMatchStateIsTransitionShiftTheHubIsActive() {
        var machine = new RebuiltStateMachine();
        HubState.setup(machine);

        CommandScheduler.getInstance().schedule(machine.transitionTo(MatchState.TRANSITION_SHIFT));

        assertEquals(MatchState.TRANSITION_SHIFT, machine.currentState().matchState());
        assertEquals(HubState.ACTIVE, machine.currentState().hubState());
    }

    @Test
    void whenTheMatchStateIsEndGameTheHubIsActive() {
        var machine = new RebuiltStateMachine(MatchState.SHIFT_4);
        HubState.setup(machine);

        CommandScheduler.getInstance().schedule(machine.transitionTo(MatchState.END_GAME));

        assertEquals(MatchState.END_GAME, machine.currentState().matchState());
        assertEquals(HubState.ACTIVE, machine.currentState().hubState());
    }

    @ParameterizedTest
    @MethodSource("matchSetups")
    void goThroughShifts(Alliance us, Alliance autoPoints) {
        setOurAlliance(us);
        setGameSpecificMessage(autoPoints);
        boolean shiftOneIsOurs = us != autoPoints;

        // Transition shift will start active
        var machine = new RebuiltStateMachine(MatchState.TRANSITION_SHIFT, HubState.ACTIVE);
        HubState.setup(machine);

        // Shift 1
        CommandScheduler.getInstance().schedule(machine.transitionTo(MatchState.SHIFT_1));
        machine.poll();

        assertEquals(shiftOneIsOurs, machine.currentState().hubState() == HubState.ACTIVE, "Shift 1 is incorrect");

        // Shift 2
        CommandScheduler.getInstance().schedule(machine.transitionTo(MatchState.SHIFT_2));
        machine.poll();

        assertEquals(shiftOneIsOurs, machine.currentState().hubState() == HubState.INACTIVE, "Shift 2 is incorrect");

        // Shift 3
        CommandScheduler.getInstance().schedule(machine.transitionTo(MatchState.SHIFT_3));
        machine.poll();

        assertEquals(shiftOneIsOurs, machine.currentState().hubState() == HubState.ACTIVE, "Shift 3 is incorrect");

        // Shift 4
        CommandScheduler.getInstance().schedule(machine.transitionTo(MatchState.SHIFT_4));
        machine.poll();

        assertEquals(shiftOneIsOurs, machine.currentState().hubState() == HubState.INACTIVE, "Shift 4 is incorrect");
    }

    @ParameterizedTest
    @EnumSource(Alliance.class)
    void emptyGameSpecificMessageMakesHubAlwaysActive(Alliance us) {
        setOurAlliance(us);
        var machine = new RebuiltStateMachine();
        HubState.setup(machine);
        TestHelpers.setGameSpecificMessage("");

        CommandScheduler.getInstance().schedule(machine.transitionTo(MatchState.AUTO));
        machine.poll();

        assertEquals(MatchState.AUTO, machine.currentState().matchState());
        assertEquals(HubState.ACTIVE, machine.currentState().hubState());

        CommandScheduler.getInstance().schedule(machine.transitionTo(MatchState.TRANSITION_SHIFT));
        machine.poll();

        assertEquals(MatchState.TRANSITION_SHIFT, machine.currentState().matchState());
        assertEquals(HubState.ACTIVE, machine.currentState().hubState());

        CommandScheduler.getInstance().schedule(machine.transitionTo(MatchState.SHIFT_1));
        machine.poll();

        assertEquals(MatchState.SHIFT_1, machine.currentState().matchState());
        assertEquals(HubState.ACTIVE, machine.currentState().hubState());

        CommandScheduler.getInstance().schedule(machine.transitionTo(MatchState.SHIFT_2));
        machine.poll();

        assertEquals(MatchState.SHIFT_2, machine.currentState().matchState());
        assertEquals(HubState.ACTIVE, machine.currentState().hubState());

        CommandScheduler.getInstance().schedule(machine.transitionTo(MatchState.SHIFT_3));
        machine.poll();

        assertEquals(MatchState.SHIFT_3, machine.currentState().matchState());
        assertEquals(HubState.ACTIVE, machine.currentState().hubState());

        CommandScheduler.getInstance().schedule(machine.transitionTo(MatchState.SHIFT_4));
        machine.poll();

        assertEquals(MatchState.SHIFT_4, machine.currentState().matchState());
        assertEquals(HubState.ACTIVE, machine.currentState().hubState());

        CommandScheduler.getInstance().schedule(machine.transitionTo(MatchState.END_GAME));
        machine.poll();

        assertEquals(MatchState.END_GAME, machine.currentState().matchState());
        assertEquals(HubState.ACTIVE, machine.currentState().hubState());
    }

    @ParameterizedTest
    @EnumSource(Alliance.class)
    void whenAllianceIsUnspecifiedHubIsAlwaysActive(Alliance autoPoints) {
        setOurAlliance(null);
        var machine = new RebuiltStateMachine();
        HubState.setup(machine);
        setGameSpecificMessage(autoPoints);

        CommandScheduler.getInstance().schedule(machine.transitionTo(MatchState.AUTO));
        machine.poll();

        assertEquals(MatchState.AUTO, machine.currentState().matchState());
        assertEquals(HubState.ACTIVE, machine.currentState().hubState());

        CommandScheduler.getInstance().schedule(machine.transitionTo(MatchState.TRANSITION_SHIFT));
        machine.poll();

        assertEquals(MatchState.TRANSITION_SHIFT, machine.currentState().matchState());
        assertEquals(HubState.ACTIVE, machine.currentState().hubState());

        CommandScheduler.getInstance().schedule(machine.transitionTo(MatchState.SHIFT_1));
        machine.poll();

        assertEquals(MatchState.SHIFT_1, machine.currentState().matchState());
        assertEquals(HubState.ACTIVE, machine.currentState().hubState());

        CommandScheduler.getInstance().schedule(machine.transitionTo(MatchState.SHIFT_2));
        machine.poll();

        assertEquals(MatchState.SHIFT_2, machine.currentState().matchState());
        assertEquals(HubState.ACTIVE, machine.currentState().hubState());

        CommandScheduler.getInstance().schedule(machine.transitionTo(MatchState.SHIFT_3));
        machine.poll();

        assertEquals(MatchState.SHIFT_3, machine.currentState().matchState());
        assertEquals(HubState.ACTIVE, machine.currentState().hubState());

        CommandScheduler.getInstance().schedule(machine.transitionTo(MatchState.SHIFT_4));
        machine.poll();

        assertEquals(MatchState.SHIFT_4, machine.currentState().matchState());
        assertEquals(HubState.ACTIVE, machine.currentState().hubState());

        CommandScheduler.getInstance().schedule(machine.transitionTo(MatchState.END_GAME));
        machine.poll();

        assertEquals(MatchState.END_GAME, machine.currentState().matchState());
        assertEquals(HubState.ACTIVE, machine.currentState().hubState());
    }

    private static Stream<Arguments> matchSetups() {
        return Arrays.stream(Alliance.values())
                .flatMap(a -> Arrays.stream(Alliance.values())
                        .map(b -> Arguments.arguments(a, b)));
    }

    private static void setOurAlliance(Alliance us) {
        if(us == Alliance.Red) {
            DriverStationSim.setAllianceStationId(AllianceStationID.Red1);
        } else if(us == Alliance.Blue) {
            DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
        } else {
            DriverStationSim.setAllianceStationId(AllianceStationID.Unknown);
        }
        DriverStationSim.notifyNewData();
    }

    private static void setGameSpecificMessage(Alliance autoPoints) {
        if(autoPoints == Alliance.Red) {
            TestHelpers.setGameSpecificMessage("R");
        } else {
            TestHelpers.setGameSpecificMessage("B");
        }
    }
}
