package frc.robot.state;

import bot.den.foxflow.DefaultState;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public enum IndexerState {
    @DefaultState INACTIVE,
    ACTIVE,
    REVERSE;

    public static void setup(RebuiltStateMachine stateMachine, BooleanSupplier rightTrigger, BooleanSupplier leftTrigger, BooleanSupplier aButton, BooleanSupplier xButton) {

    }
}
