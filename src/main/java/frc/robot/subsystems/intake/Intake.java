package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  public final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private AngularVelocity intakeVelocitySetpoint = RotationsPerSecond.of(60);
  private Angle intakeExtensionSetpoint = Rotations.zero();

  public Intake(IntakeIO io) {
    this.io = io;
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
    return Commands.runEnd(
        () -> {
          intakeVelocitySetpoint = speed;
          this.io.setIntakeVelocity(speed);}, 
          () -> this.io.stopIntake());
  }

  public Command stopIntake() {
    return Commands.runOnce(() -> this.io.stopIntake());
  }

  public Command runIntakeExtension(Angle position) {
    return Commands.runOnce(
        () -> {
          intakeExtensionSetpoint = position;
          this.io.setIntakeExtension(position);});
  }

  public Command setIntakeMaxLength() {
    return Commands.runOnce(
        () -> {
          intakeExtensionSetpoint = IntakeConstants.intakeMaxExtensionPosition;
          this.io.setIntakeMaxLength();});
  }

  public Command setIntakeIdleLength() {
    return Commands.runOnce(
        () -> {
          intakeExtensionSetpoint = IntakeConstants.intakeIdleExtensionPosition;
          this.io.setIntakeIdleLength();});
  }

  public Command setIntakeMinLength() {
    return Commands.runOnce(
        () -> {
          intakeExtensionSetpoint = IntakeConstants.intakeMinExtensionPosition;
          this.io.setIntakeMinLength();});
  }

  // Getters for private IO variables
  public boolean getIntakeMotorConnected() {
    return inputs.intakeMotorConnected;
  }

  public boolean getExtensionMotorsConnected() {
    return inputs.extensionMotorLeftConnected && inputs.extensionMotorRightConnected;
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

  public Angle getIntakePositionRots() {
    return inputs.intakePositionRots;
  }

  public Angle getExtensionLeftPositionRots() {
    return inputs.extensionLeftPositionRots;
  }
  
  public Angle getExtensionRightPositionRots() {
    return inputs.extensionRightPositionRots;
  }
}
