package frc.robot.state;

import bot.den.foxflow.DefaultState;

import java.util.function.BooleanSupplier;

public enum ShooterState {
    @DefaultState STOPPED,
    SPINNING_UP,
    AT_SPEED;

    public static void setup(RebuiltStateMachine stateMachine, BooleanSupplier rightBumper, BooleanSupplier yButton) {
        // Shooter functions
    }
}
