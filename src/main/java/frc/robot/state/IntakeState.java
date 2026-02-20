package frc.robot.state;

import bot.den.foxflow.DefaultState;

import java.util.function.BooleanSupplier;

public enum IntakeState {
    ACTIVE,
    REVERSE,
    @DefaultState INACTIVE;

    public static void setup(RebuiltStateMachine stateMachine, BooleanSupplier rightBumper, BooleanSupplier yButton) {
        // Intake functions
        stateMachine
                .state(IntakeState.INACTIVE, HopperState.DEPLOYED)
                .to(IntakeState.ACTIVE)
                .transitionWhen(rightBumper); // Transition to active when trigger is pressed
        stateMachine
                .state(IntakeState.ACTIVE, HopperState.DEPLOYED)
                .to(IntakeState.INACTIVE)
                .transitionWhen(() -> !rightBumper.getAsBoolean()); // Transition to inactive when trigger is let go
        stateMachine
                .state(HopperState.RETRACTING)
                .to(IntakeState.INACTIVE)
                .transitionAlways(); // Transition to inactive if retracting

        stateMachine
                .state(IntakeState.INACTIVE)
                .to(IntakeState.REVERSE)
                .transitionWhen(yButton); // Goes from inactive to reverse if y button is pressed
        stateMachine
                .state(IntakeState.REVERSE)
                .to(IntakeState.INACTIVE)
                .transitionWhen(() -> !yButton.getAsBoolean()); // Go from reverse to inactive if y button let go
    }
}
