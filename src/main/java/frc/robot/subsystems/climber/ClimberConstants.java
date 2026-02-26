package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;

public class ClimberConstants {
    public static final int CLIMBER_MOTOR_ID = 0; // TODO find climber motor ID
    public static final int CLIMBER_ROTATION_TO_METER_RATIO = 1; // TODO find ratio between 1 revolution of motor and 1 meter of travel
    public static final Distance CLIMBER_MAX_EXTENSION_LENGTH = Meters.of(0.762); // TODO approximate value currently, measure actual full extension height
    public static final Distance CLIMBER_MIN_EXTENSION_LENGTH = Meters.of(0.0);
}
