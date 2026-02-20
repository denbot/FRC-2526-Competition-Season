// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.path.PathConstraints;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    // Mr Ray canivore ID public static final String canivoreSerial = "D75CCE723353385320202034111303FF";
    public static final CANBus canivoreCANBus = new CANBus("33ED01F9463847532020204B451D0DFF");
    public static final PathConstraints pathfindingConstraints =
      new PathConstraints(6.0, 6.0, Units.degreesToRadians(540), Units.degreesToRadians(720));
  }

  public static class FieldDimensions {
    public static final Distance fieldLength = Meters.of(16.54);
    public static final Distance fieldWidth = Meters.of(8.07);
    public static final Distance distanceFromCorner = Meters.of(0.665988);
  }

  public static class PointsOfInterest {
    public static final Pose2d centerOfHubBlue = new Pose2d(Meters.of(4.625594), Meters.of(4.034536), new Rotation2d(0)); // Point of center of hub, in meters for blue
    public static final Pose2d centerOfHubRed = new Pose2d(Meters.of(11.914406), Meters.of(4.034536), new Rotation2d(0)); // Point of center of hub, in meters for red
    public static final Pose2d cornerSW = new Pose2d(FieldDimensions.distanceFromCorner, FieldDimensions.distanceFromCorner, new Rotation2d(0));
    public static final Pose2d cornerNW = new Pose2d(FieldDimensions.distanceFromCorner, FieldDimensions.fieldWidth.minus(FieldDimensions.distanceFromCorner), new Rotation2d(0));
    public static final Pose2d cornerSE = new Pose2d(FieldDimensions.fieldLength.minus(FieldDimensions.distanceFromCorner), FieldDimensions.distanceFromCorner, new Rotation2d(0));
    public static final Pose2d cornerNE = new Pose2d(FieldDimensions.fieldLength.minus(FieldDimensions.distanceFromCorner), FieldDimensions.fieldWidth.minus(FieldDimensions.distanceFromCorner), new Rotation2d(0));
  }
}
