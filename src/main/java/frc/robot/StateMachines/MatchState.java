package frc.robot.StateMachines;

import bot.den.foxflow.DefaultState;
import bot.den.foxflow.LimitsStateTransitions;
import bot.den.foxflow.RobotState;
import edu.wpi.first.units.measure.Time;

import java.util.Set;

import static edu.wpi.first.units.Units.Seconds;

public enum MatchState implements LimitsStateTransitions<MatchState> {
    @DefaultState NONE(0),
    AUTO(20),
    TRANSITION_SHIFT(10),
    SHIFT_1(25),
    SHIFT_2(25),
    SHIFT_3(25),
    SHIFT_4(25),
    END_GAME(30);

    public final Time timeInState;

    MatchState(int timeInState) {
        this.timeInState = Seconds.of(timeInState);
    }

    public static void setup(RebuiltStateMachine stateMachine) {
        // When auto is enabled, AUTO is also the match state
        stateMachine.state(RobotState.AUTO, NONE).to(AUTO).transitionAlways();
        // When our time is up, we have no match state
        stateMachine.state(AUTO).to(NONE).transitionAfter(MatchState.AUTO.timeInState);

        // When teleop is enabled, start the transition shift
        stateMachine.state(RobotState.TELEOP, NONE).to(TRANSITION_SHIFT).transitionAlways();
        // Progress through the various states as time progresses
        stateMachine.state(TRANSITION_SHIFT).to(SHIFT_1).transitionAfter(MatchState.TRANSITION_SHIFT.timeInState);
        stateMachine.state(SHIFT_1).to(SHIFT_2).transitionAfter(MatchState.SHIFT_1.timeInState);
        stateMachine.state(SHIFT_2).to(SHIFT_3).transitionAfter(MatchState.SHIFT_2.timeInState);
        stateMachine.state(SHIFT_3).to(SHIFT_4).transitionAfter(MatchState.SHIFT_3.timeInState);
        stateMachine.state(SHIFT_4).to(END_GAME).transitionAfter(MatchState.SHIFT_4.timeInState);
        stateMachine.state(END_GAME).to(NONE).transitionAfter(MatchState.END_GAME.timeInState);

        // TODO Requires matcher rules
        // If we move to a disabled state at any time, we no longer have a match state
//            stateMachine
//                    .state(RobotStateMatcher.except(RobotState.DISABLED), MatchStateMatcher.except(NONE))
//                    .to(RobotState.DISABLED)
//                    .transitionTo(NONE);

        // Same code as above, we just need more features on the state machine library to get that concise version
        for (var robotState : RobotState.values()) {
            if (robotState == RobotState.DISABLED) continue;
            for (var matchState : MatchState.values()) {
                if (matchState == NONE) continue;

                stateMachine.state(robotState, matchState).to(RobotState.DISABLED).transitionTo(NONE);
            }
        }
    }

    @Override
    public boolean canTransitionState(MatchState newState) {
        return (switch (this) {
            case NONE -> Set.of(AUTO, TRANSITION_SHIFT);
            case AUTO, END_GAME -> Set.of(NONE, TRANSITION_SHIFT);
            case TRANSITION_SHIFT -> Set.of(NONE, SHIFT_1);
            case SHIFT_1 -> Set.of(NONE, SHIFT_2);
            case SHIFT_2 -> Set.of(NONE, SHIFT_3);
            case SHIFT_3 -> Set.of(NONE, SHIFT_4);
            case SHIFT_4 -> Set.of(NONE, END_GAME);
        }).contains(newState);
    }
}
