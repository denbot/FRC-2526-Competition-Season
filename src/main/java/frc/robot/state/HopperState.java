package frc.robot.state;

import bot.den.foxflow.DefaultState;

import java.util.function.BooleanSupplier;

public enum HopperState {
    DEPLOYING,
    DEPLOYED,
    RETRACTING,
    @DefaultState RETRACTED;

    public static void setup(RebuiltStateMachine stateMachine, BooleanSupplier Button, BooleanSupplier RetractingLimitSwitch, BooleanSupplier DeployingLimitSwitch) {
        // Intake functions
        stateMachine
                .state(HopperState.DEPLOYED)
                .to(HopperState.RETRACTING)
                .transitionWhen(Button); // Transition to retracting when bumper is pressed
        stateMachine
                .state(HopperState.RETRACTING)
                .to(HopperState.RETRACTED)
                .transitionWhen(RetractingLimitSwitch); // Transition to retracted when limit switch is activated
        stateMachine
                .state(HopperState.RETRACTED)
                .to(HopperState.DEPLOYING)
                .transitionWhen(Button); // Transition to deploying when bumper is pressed
        stateMachine
                .state(HopperState.DEPLOYING)
                .to(HopperState.DEPLOYED)
                .transitionWhen(DeployingLimitSwitch); // Transition to deployed when limit switch is activated

    }
}

