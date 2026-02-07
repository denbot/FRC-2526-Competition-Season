package frc.robot.state;

import bot.den.foxflow.DefaultState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Set;

public enum HubState {
    @DefaultState INACTIVE,
    ACTIVE;

    private enum AutoWin {
        TRUE,
        FALSE,
        NONE;
    }

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

        if (!weWonTheAutoPoints().equals(AutoWin.NONE)) { // If we either won the auto or did not win the auto

            // Game state during shifts
            for(var shift : Set.of(MatchState.SHIFT_1, MatchState.SHIFT_3)) {
                // Deactivate shift 1 & 3 if we won the auto points
                stateMachine
                        .state(shift, HubState.ACTIVE)
                        .to(HubState.INACTIVE)
                        .transitionWhen(HubState::weWonAuto);

                // Activate shift 1 & 3 if we lost the auto points
                stateMachine
                        .state(shift, HubState.INACTIVE)
                        .to(HubState.ACTIVE)
                        .transitionWhen(() -> ! weWonAuto());
            }

            for(var shift : Set.of(MatchState.SHIFT_2, MatchState.SHIFT_4)) {
                // Activate shift 2 & 4 if we won the auto points
                stateMachine
                        .state(shift, HubState.INACTIVE)
                        .to(HubState.ACTIVE)
                        .transitionWhen(HubState::weWonAuto);

                // Deactivate shift 2 & 4 if we lost the auto points
                stateMachine
                        .state(shift, HubState.ACTIVE)
                        .to(HubState.INACTIVE)
                        .transitionWhen(() -> ! weWonAuto());
            }
        } else { // If we do not have an alliance or there was no message
            // Activate all shifts
            for(var shift : Set.of(MatchState.SHIFT_1, MatchState.SHIFT_2, MatchState.SHIFT_3, MatchState.SHIFT_4)) {
                // Activate shift 1, 2, 3 & 4 if we won the auto points
                stateMachine
                        .state(shift, HubState.INACTIVE)
                        .to(HubState.ACTIVE)
                        .transitionWhen(() -> true);
            }
        }
        
    }

    private static boolean autoWinConverter(AutoWin autoWin) {
        switch(autoWin) {
            case TRUE:
                return true;
            case FALSE:
                return false;
            case NONE: // Unreachable
                return false;
            default: // Unreachable
                return false;
        }
    }

    private static boolean weWonAuto() {
        return autoWinConverter(weWonTheAutoPoints());
    }

    private static AutoWin weWonTheAutoPoints() {
        String gameSpecificMessage = DriverStation.getGameSpecificMessage();
        var alliance = DriverStation.getAlliance().get();

        if (DriverStation.getAlliance().isEmpty() || (!gameSpecificMessage.equals("R") && !gameSpecificMessage.equals("B"))) { // If we don't have an alliance or there was not a game specific message
            SmartDashboard.putString("CAN'T CONFIGURE HUBSTATE SHIFTS", DriverStation.getAlliance().isEmpty() ? "Issue: no alliance found": "Issue: Game-specific message not given");
            return AutoWin.NONE;
        }

        if (alliance == Alliance.Red && gameSpecificMessage.equals("R")) {
            return AutoWin.TRUE;
        } else if (alliance == Alliance.Red && gameSpecificMessage.equals("B")) {
            return AutoWin.FALSE;
        } else if (alliance == Alliance.Blue && gameSpecificMessage.equals("R")) {
            return AutoWin.FALSE;
        } else if (alliance == Alliance.Blue && gameSpecificMessage.equals("B")) {
            return AutoWin.TRUE;
        }
        return AutoWin.NONE;
    }
}
