package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Angle;


public interface ShooterIO {

    @AutoLog
    public static class ShooterIOInputs {
        public boolean leftSpinnerMotorConnected = false;
        public boolean rightSpinnerMotorConnected = false;
        public boolean kickerMotorConnected = false;
        public AngularVelocity leftSpinnerRotationSpeed = RevolutionsPerSecond.zero();
        public AngularVelocity rightSpinnerRotationSpeed = RevolutionsPerSecond.zero();
        public AngularVelocity kickerRotationSpeed = RevolutionsPerSecond.zero();
        public double leftSpinnerClosedLoopError = 0.0;
        public double rightSpinnerClosedLoopError = 0.0;
        public double kickerClosedLoopError = 0.0;
        public Current leftSpinnerCurrentAmps = Amp.zero();
        public Current rightSpinnerCurrentAmps = Amp.zero();
        public Current kickerCurrentAmps = Amp.zero();
        public Angle leftSpinnerPositionRots = Degree.zero();
        public Angle rightSpinnerPositionRots = Degree.zero();
        public Angle kickerPositionRots = Degree.zero();
    }

    //sets shooter velocity in RevolutionsPerSec
    public default void updateInputs(ShooterIOInputs inputs) {}
    public default void setSpinnerVelocity(AngularVelocity velocity) {}
    public default void stopSpinner() {}
    public default void setKickerVelocity(AngularVelocity velocity) {}
    public default void stopKicker() {}
}