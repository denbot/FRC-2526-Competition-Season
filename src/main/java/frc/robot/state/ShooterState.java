package frc.robot.state;

import bot.den.foxflow.DefaultState;

import java.util.function.BooleanSupplier;

public enum ShooterState {
    @DefaultState STOPPED,
    SPINNING_UP,
    AT_SPEED;

    public static void setup(RebuiltStateMachine stateMachine, BooleanSupplier rightBumper, BooleanSupplier yButton) {
        // Shooter functions
        stateMachine
                .state(ShooterState.STOPPED)
                .to(ShooterState.SPINNING_UP)
                .transitionWhen(() -> rightBumper.getAsBoolean() || yButton.getAsBoolean());
        stateMachine
                .state(ShooterState.SPINNING_UP)
                .to(ShooterState.STOPPED)
                .transitionWhen(() -> !rightBumper.getAsBoolean() && !yButton.getAsBoolean());
        stateMachine
                .state(ShooterState.AT_SPEED)
                .to(ShooterState.STOPPED)
                .transitionWhen(() -> !rightBumper.getAsBoolean() && !yButton.getAsBoolean());
    }
}
