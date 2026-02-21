package frc.robot.state;

import bot.den.foxflow.DefaultState;

import java.util.function.BooleanSupplier;

public enum HopperState {
    @DefaultState RETRACTED,
    DEPLOYING,
    DEPLOYED,
    RETRACTING_TO_IDLE,
    IDLE,
    RETRACTING_TO_RETRACTED;

    public static void setup(RebuiltStateMachine stateMachine, BooleanSupplier leftTrigger, BooleanSupplier leftBumper, BooleanSupplier xButton) {
        // Hopper functions
        stateMachine
                .state(HopperState.RETRACTED)
                .to(HopperState.DEPLOYING)
                .transitionWhen(leftTrigger);
        stateMachine
                .state(HopperState.IDLE)
                .to(HopperState.DEPLOYING)
                .transitionWhen(leftTrigger);

        stateMachine
                .state(HopperState.DEPLOYED)
                .to(HopperState.RETRACTING_TO_IDLE)
                .transitionWhen(() -> !leftTrigger.getAsBoolean());
        stateMachine
                .state(HopperState.IDLE)
                .to(HopperState.RETRACTING_TO_RETRACTED)
                .transitionWhen(leftBumper);
    }
}

