package frc.robot.state;

import bot.den.foxflow.DefaultState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Set;
import java.util.function.Supplier;

public enum HubState {
    @DefaultState INACTIVE,
    ACTIVE;

    static HubStateFixed hubStateFixed = HubStateFixed.FALSE;
    private static boolean hubStateShiftsFixed = false;
    private static boolean weWonAutoPointsAndHubStateShiftsNotConfiguredAutomatically = false;

    public static void setup(RebuiltStateMachine stateMachine) {
        // hubStateFixed = HubStateFixed.FALSE;
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
                    .transitionWhen(() -> (hubStateShiftsConfigurable(hubStateFixed)? weWonTheAutoPoints(): (getHubStateFixed(hubStateFixed)? weWonAutoPointsAndHubStateShiftsNotConfiguredAutomatically: false)));

            // Activate shift 1 & 3 if we lost the auto points
            stateMachine
                    .state(shift, HubState.INACTIVE)
                    .to(HubState.ACTIVE)
                    .transitionWhen(() -> (hubStateShiftsConfigurable(hubStateFixed)? !weWonTheAutoPoints(): (getHubStateFixed(hubStateFixed)? !weWonAutoPointsAndHubStateShiftsNotConfiguredAutomatically: false)));
        }

        for(var shift : Set.of(MatchState.SHIFT_2, MatchState.SHIFT_4)) {
            // Activate shift 2 & 4 if we won the auto points
            stateMachine
                    .state(shift, HubState.INACTIVE)
                    .to(HubState.ACTIVE)
                    .transitionWhen(() -> (hubStateShiftsConfigurable(hubStateFixed)? weWonTheAutoPoints(): (getHubStateFixed(hubStateFixed)? weWonAutoPointsAndHubStateShiftsNotConfiguredAutomatically: false)));

            // Deactivate shift 2 & 4 if we lost the auto points
            stateMachine
                    .state(shift, HubState.ACTIVE)
                    .to(HubState.INACTIVE)
                    .transitionWhen(() -> (hubStateShiftsConfigurable(hubStateFixed)? !weWonTheAutoPoints(): (getHubStateFixed(hubStateFixed)? !weWonAutoPointsAndHubStateShiftsNotConfiguredAutomatically: false)));
        }
    }

    public static void configureShifts(boolean weWonAutoPoints) {
        hubStateFixed = HubStateFixed.TRUE;
        weWonAutoPointsAndHubStateShiftsNotConfiguredAutomatically = weWonAutoPoints;
    }

    private static boolean weWonTheAutoPoints() {
        try {
            String gameSpecificMessage = DriverStation.getGameSpecificMessage();
            var alliance = DriverStation.getAlliance().get();

            if (alliance == Alliance.Red && gameSpecificMessage.equals("R")) {
                return true;
            } else if (alliance == Alliance.Red && gameSpecificMessage.equals("B")) {
                return false;
            } else if (alliance == Alliance.Blue && gameSpecificMessage.equals("R")) {
                return false;
            } else {
                return (alliance == Alliance.Blue && gameSpecificMessage.equals("B"));
            }
        } catch (Exception e) {
            System.out.println(e.getStackTrace());
        }
        return false;
    }

    private static boolean hubStateShiftsConfigurable(HubStateFixed hubStateShiftsFixedSupplier) {
        switch(hubStateShiftsFixedSupplier) {
            case TRUE:
                return false;
            case FALSE:
                String gameSpecificMessage = DriverStation.getGameSpecificMessage();
                if (DriverStation.getAlliance().isEmpty() || (!gameSpecificMessage.equals("R") && !gameSpecificMessage.equals("B"))) { // If we don't have an alliance or there was not a game specific message
                    System.out.println("iwbnfiewnrifew3rf");
                    SmartDashboard.putString("CAN'T CONFIGURE HUBSTATE SHIFTS", DriverStation.getAlliance().isEmpty() ? "Issue: no alliance found": "Issue: Game-specific message not given");
                    return false;
                }
                return true;
            default:
                return false;
        }
    }

    private static boolean getHubStateFixed(HubStateFixed hubStateShiftsFixedSupplier) {
        switch(hubStateShiftsFixedSupplier) {
            case TRUE:
                return false;
            case FALSE:
                return false;
            default:
                return false;
        }
    }
}
