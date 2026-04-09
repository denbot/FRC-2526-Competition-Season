package frc.robot.state;

import bot.den.foxflow.DefaultState;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public enum IndexerState {
    @DefaultState STOPPED,
    RUNNING,
    REVERSING;

    public static void setup(RebuiltStateMachine stateMachine, BooleanSupplier rightTrigger, BooleanSupplier leftTrigger, BooleanSupplier aButton, BooleanSupplier xButton, BooleanSupplier churnSwitch) {
        stateMachine
                .state(IndexerState.STOPPED)
                .to(IndexerState.RUNNING)
                .transitionWhen(leftTrigger); // Transition to running when trigger is pressed
        stateMachine
                .state(IndexerState.STOPPED, ShooterState.AT_SPEED)
                .to(IndexerState.RUNNING)
                .transitionWhen(rightTrigger); // Transition to running when trigger is pressed     
        stateMachine
                .state(IndexerState.RUNNING)
                .to(IndexerState.STOPPED)
                .transitionWhen(() -> (!leftTrigger.getAsBoolean() && !rightTrigger.getAsBoolean())); // Transition to inactive when trigger is let go
        stateMachine
                .state(IndexerState.RUNNING, ShooterState.STOPPED)
                .to(IndexerState.STOPPED)
                .transitionWhen(() -> (!leftTrigger.getAsBoolean())); // Transition to inactive when left trigger is let go and shooter isn't at speed
        stateMachine
                .state(IndexerState.RUNNING, ShooterState.SPINNING_UP_ADAPTIVE)
                .to(IndexerState.STOPPED)
                .transitionWhen(() -> (!leftTrigger.getAsBoolean())); // Transition to inactive when left trigger is let go and shooter isn't at speed
        stateMachine
                .state(IndexerState.RUNNING, ShooterState.SPINNING_UP_FIXED)
                .to(IndexerState.STOPPED)
                .transitionWhen(() -> (!leftTrigger.getAsBoolean())); // Transition to inactive when left trigger is let go and shooter isn't at speed

        stateMachine
                .state(IndexerState.STOPPED)
                .to(IndexerState.REVERSING)
                .transitionWhen(() -> aButton.getAsBoolean() || xButton.getAsBoolean() || churnSwitch.getAsBoolean()); // Goes from inactive to reverse if x button is pressed
        stateMachine
                .state(IndexerState.REVERSING)
                .to(IndexerState.STOPPED)
                .transitionWhen(() -> !aButton.getAsBoolean() && !xButton.getAsBoolean() && !churnSwitch.getAsBoolean()); // Go from reverse to inactive if x button let go
    }
}
