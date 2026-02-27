package frc.robot.state;

import bot.den.foxflow.DefaultState;

import java.util.function.BooleanSupplier;

public enum IntakeState {
    @DefaultState STOPPED,
    RUNNING,
    REVERSING;


    public static void setup(RebuiltStateMachine stateMachine, BooleanSupplier leftTrigger, BooleanSupplier xButton) {
        // Intake functions
        stateMachine
                .state(IntakeState.STOPPED)
                .to(IntakeState.RUNNING)
                .transitionWhen(leftTrigger); // Transition to running when trigger is pressed
        stateMachine
                .state(IntakeState.RUNNING)
                .to(IntakeState.STOPPED)
                .transitionWhen(() -> !leftTrigger.getAsBoolean()); // Transition to inactive when trigger is let go

        stateMachine
                .state(IntakeState.STOPPED)
                .to(IntakeState.REVERSING)
                .transitionWhen(xButton); // Goes from inactive to reverse if x button is pressed
        stateMachine
                .state(IntakeState.REVERSING)
                .to(IntakeState.STOPPED)
                .transitionWhen(() -> !xButton.getAsBoolean()); // Go from reverse to inactive if x button let go
    }
}
