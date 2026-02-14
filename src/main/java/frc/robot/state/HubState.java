package frc.robot.state;

import bot.den.foxflow.DefaultState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;

public enum HubState {
    @DefaultState INACTIVE,
    ACTIVE;

    public static void setup(RebuiltStateMachine stateMachine, Supplier<Optional<Boolean>> operatorOverride) {
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
                    .transitionWhen(() -> {
                        var weWonTheAutoPoints = weWonTheAutoPoints();
                        if (weWonTheAutoPoints.isEmpty()) {
                            if (operatorOverride.get().isEmpty()) {
                                return false;
                            } else {
                                return operatorOverride.get().get();
                            }
                        } else return weWonTheAutoPoints.get();
                    });

            // Activate shift 1 & 3 if we lost the auto points
            stateMachine
                    .state(shift, HubState.INACTIVE)
                    .to(HubState.ACTIVE)
                    .transitionWhen(() -> {
                        var weWonTheAutoPoints = weWonTheAutoPoints();
                        if (weWonTheAutoPoints.isEmpty()) {
                            if (operatorOverride.get().isEmpty()) {
                                return false;
                            } else {
                                return !operatorOverride.get().get();
                            }
                        } else return !weWonTheAutoPoints.get();
                    });
        }

        for(var shift : Set.of(MatchState.SHIFT_2, MatchState.SHIFT_4)) {
            // Activate shift 2 & 4 if we won the auto points
            stateMachine
                    .state(shift, HubState.INACTIVE)
                    .to(HubState.ACTIVE)
                    .transitionWhen(() -> {
                        var weWonTheAutoPoints = weWonTheAutoPoints();
                        if (weWonTheAutoPoints.isEmpty()) {
                            if (operatorOverride.get().isEmpty()) {
                                return false;
                            } else {
                                return operatorOverride.get().get();
                            }
                        } else return weWonTheAutoPoints.get();
                    });

            // Deactivate shift 2 & 4 if we lost the auto points
            stateMachine
                    .state(shift, HubState.ACTIVE)
                    .to(HubState.INACTIVE)
                    .transitionWhen(() -> {
                        var weWonTheAutoPoints = weWonTheAutoPoints();
                        if (weWonTheAutoPoints.isEmpty()) {
                            if (operatorOverride.get().isEmpty()) {
                                return false;
                            } else {
                                return !operatorOverride.get().get();
                            }
                        } else return !weWonTheAutoPoints.get();
                    });
        }
    }

    private static Optional<Boolean> weWonTheAutoPoints() {
        if (DriverStation.getAlliance().isEmpty()) {
            return Optional.empty();
        }

        String gameSpecificMessage = DriverStation.getGameSpecificMessage();
        var alliance = DriverStation.getAlliance().get();
        if (alliance == Alliance.Red && gameSpecificMessage.equals("R")) {
            return Optional.of(true);
        } else if (alliance == Alliance.Red && gameSpecificMessage.equals("B")) {
            return Optional.of(false);
        } else if (alliance == Alliance.Blue && gameSpecificMessage.equals("R")) {
            return Optional.of(false);
        } else if (alliance == Alliance.Blue && gameSpecificMessage.equals("B")) {
            return Optional.of(true);
        } else {
            return Optional.empty();
        }
    }
}
