package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.state.HopperState;
import frc.robot.state.IntakeState;
import frc.robot.state.RebuiltStateMachine;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

public class Intake extends SubsystemBase {
  public final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private AngularVelocity intakeVelocitySetpoint = RotationsPerSecond.of(80);
  private Angle intakeExtensionSetpoint = Rotations.zero();

  public Intake(IntakeIO io, RebuiltStateMachine stateMachine) {
    this.io = io;

    stateMachine
            .state(HopperState.IDLE)
            .to(HopperState.DEPLOYING)
            .run(setIntakeMaxLength());
    stateMachine
            .state(HopperState.RETRACTED)
            .to(HopperState.DEPLOYING)
            .run(setIntakeMaxLength());
    stateMachine
            .state(HopperState.RETRACTING_TO_IDLE)
            .to(HopperState.DEPLOYING)
            .run(setIntakeMaxLength());
    stateMachine
            .state(HopperState.RETRACTING_TO_RETRACTED)
            .to(HopperState.DEPLOYING)
            .run(setIntakeMaxLength());
    stateMachine
            .state(HopperState.DEPLOYED)
            .to(HopperState.RETRACTING_TO_IDLE)
            .run(setIntakeIdleLength());
    stateMachine
            .state(HopperState.DEPLOYING)
            .to(HopperState.RETRACTING_TO_IDLE)
            .run(setIntakeIdleLength());
    stateMachine
            .state(HopperState.RETRACTING_TO_RETRACTED)
            .to(HopperState.RETRACTING_TO_IDLE)
            .run(setIntakeIdleLength());
    stateMachine
            .state(HopperState.RETRACTED)
            .to(HopperState.RETRACTING_TO_IDLE)
            .run(setIntakeIdleLength());
    stateMachine
            .state(HopperState.DEPLOYING)
            .to(HopperState.RETRACTING_TO_RETRACTED)
            .run(setIntakeMinLength());
    stateMachine
            .state(HopperState.DEPLOYED)
            .to(HopperState.RETRACTING_TO_RETRACTED)
            .run(setIntakeMinLength());
    stateMachine
            .state(HopperState.RETRACTING_TO_IDLE)
            .to(HopperState.RETRACTING_TO_RETRACTED)
            .run(setIntakeMinLength());
    stateMachine
            .state(HopperState.IDLE)
            .to(HopperState.RETRACTING_TO_RETRACTED)
            .run(setIntakeMinLength());

    stateMachine
            .state(IntakeState.STOPPED)
            .to(IntakeState.RUNNING)
            .run(runIntake(RotationsPerSecond.of(60)));

    stateMachine
            .state(IntakeState.STOPPED)
            .to(IntakeState.REVERSING)
            .run(runIntake(RotationsPerSecond.of(-60)));

    stateMachine
            .state(IntakeState.RUNNING)
            .to(IntakeState.STOPPED)
            .run(stopIntake());

    stateMachine
            .state(IntakeState.REVERSING)
            .to(IntakeState.STOPPED)
            .run(stopIntake());

    stateMachine
            .state(HopperState.DEPLOYING)
            .to(HopperState.DEPLOYED)
            .transitionWhen(() -> Math.abs(inputs.extensionClosedLoopError) < 0.05);

    stateMachine
            .state(HopperState.RETRACTING_TO_IDLE)
            .to(HopperState.IDLE)
            .transitionWhen(() -> Math.abs(inputs.extensionClosedLoopError) < 0.05);

    stateMachine
            .state(HopperState.RETRACTING_TO_RETRACTED)
            .to(HopperState.RETRACTED)
            .transitionWhen(() -> Math.abs(inputs.extensionClosedLoopError) < 0.05);
    // TODO Also check extensionRightPositionRots
  }

  @Override
  public void periodic() {
    // Log key variables
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    Logger.recordOutput("Intake Velocity Setpoint", intakeVelocitySetpoint);
    Logger.recordOutput("Intake Extension Setpoint", intakeExtensionSetpoint);
  }

  public Command runIntake(AngularVelocity speed) {
    return Commands.runOnce(
        () -> {
          intakeVelocitySetpoint = speed;
          this.io.setIntakeVelocity(speed);});
  }

  public Command stopIntake() {
    return Commands.runOnce(() -> {
      intakeVelocitySetpoint = RotationsPerSecond.of(0);
      this.io.stopIntake();});
  }

  public Command runIntakeExtension(Angle position) {
    return Commands.runOnce(
        () -> {
          intakeExtensionSetpoint = position;
          this.io.setIntakeExtension(position);});
  }

  public Command setIntakeMaxLength() {
    return Commands.runOnce(() -> {
          intakeExtensionSetpoint = IntakeConstants.intakeMaxExtensionPosition;
          this.io.setIntakeMaxLength();
    });
  }

  public Command setIntakeMinLength() {
    return Commands.runOnce(() -> {
          intakeExtensionSetpoint = IntakeConstants.intakeMinExtensionPosition;
          this.io.setIntakeMinLength();
    });
  }

  public Command setIntakeIdleLength() {
    return Commands.runOnce(() -> {
          intakeExtensionSetpoint = IntakeConstants.intakeIdleExtensionPosition;
          this.io.setIntakeIdleLength();});
  }

  // Getters for private IO variables
  public boolean getIntakeMotorConnected() {
    return inputs.intakeMotorConnected;
  }

  public boolean getExtensionMotorsConnected() {
    return inputs.extensionMotorLeftConnected && inputs.extensionMotorRightConnected;
  }

  public double getClosedLoopError() {
    return inputs.intakeVelocityClosedLoopError;
  }

  public boolean getIntakeDeployedSwitch() {
    return inputs.intakeDeployedSwitch;
  }

  public boolean getIntakeRetractedSwitch() {
    return inputs.intakeRetractedSwitch;
  }

  public Current getStallCurrentIntake() {
    return inputs.stallCurrentIntake;
  }

  public Current getStallCurrentExtensionLeft() {
    return inputs.stallCurrentExtensionLeft;
  }
  
  public Current getStallCurrentExtensionRight() {
    return inputs.stallCurrentExtensionRight;
  }

  public AngularVelocity getIntakeVelocityRotPerSec() {
    return inputs.intakeVelocityRotPerSec;
  }

  public AngularVelocity getExtensionLeftVelocityRotPerSec() {
    return inputs.extensionVelocityLeft;
  }
  
  public AngularVelocity getExtensionRightVelocityRotPerSec() {
    return inputs.extensionVelocityRight;
  }

  public double getExtensionClosedLoopError(){
    return inputs.extensionClosedLoopError;
  }

  public Angle getIntakePositionRots() {
    return inputs.intakePositionRots;
  }

  public Angle getExtensionLeftPositionRots() {
    return inputs.extensionLeftPositionRots;
  }
  
  public Angle getExtensionRightPositionRots() {
    return inputs.extensionRightPositionRots;
  }

  public void startJingle(){
    this.io.startJingle();
  }

  public void startIntermission(){
    this.io.startIntermission();
  }

  public void stopJingle(){
    this.io.stopJingle();
  }

  public void stopIntermission(){
    this.io.stopIntermission();
  }
}
