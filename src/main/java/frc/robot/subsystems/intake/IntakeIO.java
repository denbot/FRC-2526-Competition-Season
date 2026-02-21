package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public boolean intakeDeployedSwitch = false;
    public boolean intakeRetractedSwitch = false;
    public boolean intakeMotorConnected = false;
    public boolean extensionMotorLeftConnected = false;
    public boolean extensionMotorRightConnected = false;
    public Current stallCurrentExtensionLeft = Amp.zero();
    public Current stallCurrentExtensionRight = Amp.zero();
    public Current stallCurrentIntake = Amp.zero();
    public AngularVelocity intakeVelocityRotPerSec = RevolutionsPerSecond.zero();
    public AngularVelocity extensionVelocityLeft = RevolutionsPerSecond.zero();
    public AngularVelocity extensionVelocityRight = RevolutionsPerSecond.zero();
    public Angle intakePositionRots = Degree.zero();
    public Angle extensionLeftPositionRots = Degree.zero();
    public Angle extensionRightPositionRots = Degree.zero();
  }

  // sets shooter velocity in RevolutionsPerSec
  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setIntakeVelocity(AngularVelocity velocity) {}

  public default void setIntakeExtension(Angle position) {}

  public default void setIntakeMaxLength() {}

  public default void setIntakeIdleLength() {}

  public default void setIntakeMinLength() {}

  public default void stopIntake() {}

}
