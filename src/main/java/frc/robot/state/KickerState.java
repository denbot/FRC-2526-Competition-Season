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
    }
}
