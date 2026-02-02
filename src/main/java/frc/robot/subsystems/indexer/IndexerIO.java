package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;

public interface IndexerIO {
    @AutoLog
    public class IndexerIOInputs{
        public boolean indexMotorConnected = false;
        public AngularVelocity indexMotorRotationSpeed = RotationsPerSecond.zero();
        public double indexMotorClosedLoopError = 0.0;
        public Current indexMotorCurrentAmps = Amp.zero();
        public Angle indexMotorPositionRots = Degree.zero();
        public Temperature indexMotorTemperature = Celsius.zero();
    }

    public default void updateInputs(IndexerIOInputs inputs){}
    public default void runIndexerAtSpeed(AngularVelocity speed){}
    public default void stopIndexer(){}
}
