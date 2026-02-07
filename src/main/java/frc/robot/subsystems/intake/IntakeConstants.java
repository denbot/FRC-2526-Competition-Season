package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;

public class IntakeConstants {
  public static final double intakeRotationsToRackRatio =
      1.0; //TODO Unkown ratio for the rotations of the motors (pinion) to the surface speed of the
  // intake (rack). Units to be determined.
  public static final double intakeMaxExtensionLength = 12; // TODO: find extension max
  public static final double intakeMinExtensionLength = 0; // TODO: find extension min
  public static int INTAKE_MOTOR_ID = 40;
  public static int RACK_MOTOR_ID = 34;
  public static int DEPLOYED_DIO_PORT = 0; // TODO: find port
  public static int RETRACTED_DIO_PORT = 1; // TODO: find port
  public static final AngularVelocity intakeSpeed = RotationsPerSecond.of(60);
}
