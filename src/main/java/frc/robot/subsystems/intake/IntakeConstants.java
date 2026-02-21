package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class IntakeConstants {
  public static final Angle intakeMaxExtensionPosition = Rotations.of(0.21);
  public static final Angle intakeIdleExtensionPosition = Rotations.of(0.09);
  public static final Angle intakeMinExtensionPosition = Rotations.zero();
  public static int INTAKE_MOTOR_ID = 40;
  public static int EXTENSION_MOTOR_LEFT_ID = 35;
  public static int EXTENSION_MOTOR_RIGHT_ID = 34;
  public static int EXTENSION_GEAR_RATIO = 23;
  public static int DEPLOYED_DIO_PORT = 0; // TODO: find port
  public static int RETRACTED_DIO_PORT = 1; // TODO: find port
  public static final AngularVelocity intakeSpeed = RotationsPerSecond.of(60);
}
