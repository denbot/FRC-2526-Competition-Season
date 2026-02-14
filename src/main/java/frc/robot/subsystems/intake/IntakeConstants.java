package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class IntakeConstants {
  public static final Angle intakeMaxExtensionPosition = Rotations.of(0.2); // TODO: find extension max
  public static final Angle intakeIdleExtensionPosition = Rotations.of(0.15); // TODO: find extension max
  public static final Angle intakeMinExtensionPosition = Rotations.zero(); // TODO: find extension min
  public static int INTAKE_MOTOR_ID = 40;
  public static int RACK_MOTOR_ID = 34;
  public static int DEPLOYED_DIO_PORT = 0; // TODO: find port
  public static int RETRACTED_DIO_PORT = 1; // TODO: find port
  public static final AngularVelocity intakeSpeed = RotationsPerSecond.of(60);
}
