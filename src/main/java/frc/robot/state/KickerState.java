package frc.robot.state;

import bot.den.foxflow.DefaultState;
import com.fasterxml.jackson.databind.ser.std.BooleanSerializer;

import java.util.function.BooleanSupplier;

public enum KickerState {
    @DefaultState STOPPED,
    RUNNING,
    REVERSING;

    public static void setup(RebuiltStateMachine stateMachine, BooleanSupplier rightTrigger, BooleanSupplier leftTrigger, BooleanSupplier xButton) {
        // Kicker functions
        stateMachine
                .state(KickerState.STOPPED)
                .to(KickerState.REVERSING)
                .transitionWhen(() -> leftTrigger.getAsBoolean() || xButton.getAsBoolean());
        stateMachine
                .state(KickerState.REVERSING)
                .to(KickerState.STOPPED)
                .transitionWhen(() -> !leftTrigger.getAsBoolean() && !xButton.getAsBoolean());

        stateMachine
                .state(ShooterState.AT_SPEED, KickerState.STOPPED)
                .to(KickerState.RUNNING)
                .transitionWhen(rightTrigger);

        stateMachine
                .state(KickerState.RUNNING)
                .to(KickerState.STOPPED)
                .transitionWhen(() -> !rightTrigger.getAsBoolean());
        stateMachine
                .state(ShooterState.STOPPED, KickerState.RUNNING)
                .to(KickerState.STOPPED)
                .transitionAlways();
        stateMachine
                .state(ShooterState.SPINNING_UP_ADAPTIVE, KickerState.RUNNING)
                .to(KickerState.STOPPED)
                .transitionAlways();
        stateMachine
                .state(ShooterState.SPINNING_UP_FIXED, KickerState.RUNNING)
                .to(KickerState.STOPPED)
                .transitionAlways();
    }
}
