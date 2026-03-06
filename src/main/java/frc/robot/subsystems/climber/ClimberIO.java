package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;

import static edu.wpi.first.units.Units.Amp;

import static edu.wpi.first.units.Units.Degree;

public interface ClimberIO {
    @AutoLog    
    public class ClimberIOInputs{
        public boolean climberMotorConnected = false;
        public Angle climberMotorPosition = Degree.zero();
        public Current climberMotorAmps = Amp.zero();
        public double climberMotorClosedLoopError = 0.0;
    }

    public default void updateInputs(ClimberIOInputs inputs){}
    public default void setClimberToSetpoint(Distance setpoint){}
    public default void setClimberMaxExtension(){}
    public default void setClimberMinExtension(){}
}
