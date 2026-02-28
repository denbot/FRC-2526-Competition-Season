package frc.robot.state;

import bot.den.foxflow.DefaultState;
import com.fasterxml.jackson.databind.ser.std.BooleanSerializer;

import java.util.function.BooleanSupplier;

public enum KickerState {
    @DefaultState STOPPED,
    SPINNING_UP,
    READY_TO_SHOOT,
    SHOOTING;

    public static void setup(RebuiltStateMachine stateMachine, BooleanSupplier leftTrigger, BooleanSupplier xButton) {
        // Kicker functions
    }
}
