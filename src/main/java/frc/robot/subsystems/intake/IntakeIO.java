package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public boolean intakeDeployedSwitch = false;
    public boolean intakeRetractedSwitch = false;
    public boolean intakeMotorConnected = false;
    public boolean rackMotorConnected = false;
    public Current stallCurrentRack = Amp.zero();
    public Current stallCurrentIntake = Amp.zero();
    public AngularVelocity intakeVelocityRotPerSec = RevolutionsPerSecond.zero();
    public AngularVelocity rackVelocityRotPerSec = RevolutionsPerSecond.zero();
    public Angle intakePositionRots = Degree.zero();
    public Angle rackPositionRots = Degree.zero();
    public Distance intakeExtensionLength = Inches.of(0);
  }

  // sets shooter velocity in RevolutionsPerSec
  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setIntakeVelocity(AngularVelocity velocity) {}

  public default void setIntakeExtensionLength(Distance length) {}

  public default void setIntakeMaxLength() {}

  public default void setIntakeMinLength() {}

  public default void stopIntake() {}

  public default void stopIntakeExtension() {}

}
