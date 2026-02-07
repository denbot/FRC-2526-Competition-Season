package frc.robot.state;

import bot.den.foxflow.DefaultState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import java.util.Set;

public enum HubState {
    @DefaultState INACTIVE,
    ACTIVE;

    public static void setup(RebuiltStateMachine stateMachine) {
        // Auto
        stateMachine.state(MatchState.NONE).to(MatchState.AUTO).transitionTo(HubState.ACTIVE);

        // Transition Shift
        stateMachine.state(MatchState.NONE).to(MatchState.TRANSITION_SHIFT).transitionTo(HubState.ACTIVE);

        // End Game
        stateMachine.state(MatchState.SHIFT_4).to(MatchState.END_GAME).transitionTo(HubState.ACTIVE);

        // Any time the match stops, the hub is inactive
        for (var matchState : MatchState.values()) {
            if (matchState == MatchState.NONE) continue;

            stateMachine.state(matchState).to(MatchState.NONE).transitionTo(HubState.INACTIVE);
        }

        // Game state during shifts
        for(var shift : Set.of(MatchState.SHIFT_1, MatchState.SHIFT_3)) {
            // Deactivate shift 1 & 3 if we won the auto points
            stateMachine
                    .state(shift, HubState.ACTIVE)
                    .to(HubState.INACTIVE)
                    .transitionWhen(HubState::weWonTheAutoPoints);

            // Activate shift 1 & 3 if we lost the auto points
            stateMachine
                    .state(shift, HubState.INACTIVE)
                    .to(HubState.ACTIVE)
                    .transitionWhen(() -> ! weWonTheAutoPoints());
        }

        for(var shift : Set.of(MatchState.SHIFT_2, MatchState.SHIFT_4)) {
            // Activate shift 2 & 4 if we won the auto points
            stateMachine
                    .state(shift, HubState.INACTIVE)
                    .to(HubState.ACTIVE)
                    .transitionWhen(HubState::weWonTheAutoPoints);

            // Deactivate shift 2 & 4 if we lost the auto points
            stateMachine
                    .state(shift, HubState.ACTIVE)
                    .to(HubState.INACTIVE)
                    .transitionWhen(() -> ! weWonTheAutoPoints());
        }
    }

    private static boolean weWonTheAutoPoints() {
        if (DriverStation.getAlliance().isEmpty()) {
            return false;
        }

        String gameSpecificMessage = DriverStation.getGameSpecificMessage();
        var alliance = DriverStation.getAlliance().get();
        if (alliance == Alliance.Red && gameSpecificMessage.equals("R")) {
            return true;
        } else if (alliance == Alliance.Red && gameSpecificMessage.equals("B")) {
            return false;
        } else if (alliance == Alliance.Blue && gameSpecificMessage.equals("R")) {
            return false;
        } else return alliance == Alliance.Blue && gameSpecificMessage.equals("B");
    }
}
