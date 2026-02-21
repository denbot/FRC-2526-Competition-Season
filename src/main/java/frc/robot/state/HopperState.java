package frc.robot.state;

import bot.den.foxflow.DefaultState;

import java.util.function.BooleanSupplier;

public enum HopperState {
    DEPLOYING,
    DEPLOYED,
    RETRACTING,
    @DefaultState RETRACTED;

    public static void setup(RebuiltStateMachine stateMachine, BooleanSupplier button, BooleanSupplier retractingLimitSwitch, BooleanSupplier deployingLimitSwitch) {
        // Intake functions
        stateMachine
                .state(HopperState.DEPLOYED)
                .to(HopperState.RETRACTING)
                .transitionWhen(button); // Transition to retracting when bumper is pressed
        stateMachine
                .state(HopperState.RETRACTING)
                .to(HopperState.RETRACTED)
                .transitionWhen(retractingLimitSwitch); // Transition to retracted when limit switch is activated
        stateMachine
                .state(HopperState.RETRACTED)
                .to(HopperState.DEPLOYING)
                .transitionWhen(button); // Transition to deploying when bumper is pressed
        stateMachine
                .state(HopperState.DEPLOYING)
                .to(HopperState.DEPLOYED)
                .transitionWhen(deployingLimitSwitch); // Transition to deployed when limit switch is activated

    }
}

