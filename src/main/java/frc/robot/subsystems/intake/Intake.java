package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  public final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private AngularVelocity intakeVelocitySetpoint = RotationsPerSecond.of(60);
  private Distance intakeExtensionSetpoint = Inches.of(0);

  public Intake(IntakeIO io) {
    this.io = io;
  }

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

  public Command runIntakeExtension(Distance length) {
    return Commands.runOnce(
        () -> {
          intakeExtensionSetpoint = length;
          this.io.setIntakeExtensionLength(length);});
  }

  public Command setIntakeMaxLength() {
    return Commands.runOnce(
        () -> {
          intakeExtensionSetpoint = Meters.of(IntakeConstants.intakeRotationsToRackRatio * IntakeConstants.intakeMaxExtensionLength);
          this.io.setIntakeMaxLength();});
  }

  public Command setIntakeMinLength() {
    return Commands.runOnce(
        () -> {
          intakeExtensionSetpoint = Meters.of(IntakeConstants.intakeRotationsToRackRatio * IntakeConstants.intakeMinExtensionLength);
          this.io.setIntakeMinLength();});
  }

  // Getters for private IO variables
  public boolean getIntakeMotorConnected() {
    return inputs.intakeMotorConnected;
  }

  public boolean getRackMotorConnected() {
    return inputs.rackMotorConnected;
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

  public Current getStallCurrentRack() {
    return inputs.stallCurrentRack;
  }

  public AngularVelocity getIntakeVelocityRotPerSec() {
    return inputs.intakeVelocityRotPerSec;
  }

  public AngularVelocity getRackVelocityRotPerSec() {
    return inputs.rackVelocityRotPerSec;
  }

  public Angle getIntakePositionRots() {
    return inputs.intakePositionRots;
  }

  public Angle getRackPositionRots() {
    return inputs.rackPositionRots;
  }

  public Distance getIntakeExtensionLength() {
    return inputs.intakeExtensionLength;
  }
}
