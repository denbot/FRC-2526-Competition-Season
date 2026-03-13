package frc.robot.state;

import bot.den.foxflow.DefaultState;

import java.util.function.BooleanSupplier;

public enum ShooterState {
    @DefaultState STOPPED,
    SPINNING_UP_FIXED,
    SPINNING_UP_ADAPTIVE,
    AT_SPEED;

    public static void setup(RebuiltStateMachine stateMachine, BooleanSupplier rightBumper, BooleanSupplier yButton, BooleanSupplier bButton) {
        // Shooter functions
        stateMachine
                .state(ShooterState.STOPPED)
                .to(ShooterState.SPINNING_UP_ADAPTIVE)
                .transitionWhen(rightBumper);
        stateMachine
                .state(ShooterState.STOPPED)
                .to(ShooterState.SPINNING_UP_FIXED)
                .transitionWhen(() -> yButton.getAsBoolean() || bButton.getAsBoolean());
        stateMachine
                .state(ShooterState.SPINNING_UP_ADAPTIVE)
                .to(ShooterState.STOPPED)
                .transitionWhen(() -> !rightBumper.getAsBoolean());
        stateMachine
                .state(ShooterState.SPINNING_UP_FIXED)
                .to(ShooterState.STOPPED)
                .transitionWhen(() -> !yButton.getAsBoolean() && !bButton.getAsBoolean());
        stateMachine
                .state(ShooterState.AT_SPEED)
                .to(ShooterState.STOPPED)
                .transitionWhen(() -> !rightBumper.getAsBoolean() && !yButton.getAsBoolean() && !bButton.getAsBoolean());
    }
}
