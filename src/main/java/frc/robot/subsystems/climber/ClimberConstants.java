package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;

public class ClimberConstants {
    public static final int CLIMBER_MOTOR_ID = 20; 
    public static final int CLIMBER_ROTATION_RATIO = 45;
    public static final Angle CLIMBER_MAX_EXTENSION_SETPOINT = Rotations.of(10); // TODO Find the position in rotations of the motor 
    public static final Angle CLIMBER_MIN_EXTENSION_SETPOINT = Rotations.of(0.0);
}
