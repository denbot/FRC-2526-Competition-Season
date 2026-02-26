package frc.robot.state;

import bot.den.foxflow.DefaultState;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public enum IndexerState {
    @DefaultState STOPPED,
    RUNNING,
    REVERSING;

    public static void setup(RebuiltStateMachine stateMachine, BooleanSupplier rightTrigger, BooleanSupplier leftTrigger, BooleanSupplier aButton, BooleanSupplier xButton) {
        stateMachine
                .state(IndexerState.STOPPED)
                .to(IndexerState.RUNNING)
                .transitionWhen(leftTrigger); // Transition to running when trigger is pressed
        stateMachine
                .state(IndexerState.RUNNING)
                .to(IndexerState.STOPPED)
                .transitionWhen(() -> !leftTrigger.getAsBoolean() && !rightTrigger.getAsBoolean()); // Transition to inactive when trigger is let go

        stateMachine
                .state(IndexerState.STOPPED)
                .to(IndexerState.RUNNING)
                .transitionWhen(rightTrigger); // Transition to running when trigger is pressed
        stateMachine
                .state(IndexerState.RUNNING)
                .to(IndexerState.STOPPED)
                .transitionWhen(() -> (!rightTrigger.getAsBoolean() && !leftTrigger.getAsBoolean())); // Transition to inactive when trigger is let go

        stateMachine
                .state(IndexerState.STOPPED)
                .to(IndexerState.REVERSING)
                .transitionWhen(aButton); // Goes from inactive to reverse if x button is pressed
        stateMachine
                .state(IndexerState.REVERSING)
                .to(IndexerState.STOPPED)
                .transitionWhen(() -> !aButton.getAsBoolean() && !xButton.getAsBoolean()); // Go from reverse to inactive if x button let go

        stateMachine
                .state(IndexerState.STOPPED)
                .to(IndexerState.REVERSING)
                .transitionWhen(xButton); // Goes from inactive to reverse if x button is pressed
        stateMachine
                .state(IndexerState.REVERSING)
                .to(IndexerState.STOPPED)
                .transitionWhen(() -> !xButton.getAsBoolean() && !aButton.getAsBoolean()); // Go from reverse to inactive if x button let go
    }
}
