// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;

import static edu.wpi.first.units.Units.Meters;

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
    public static final String canivoreSerial = "D75CCE723353385320202034111303FF";
  }

  public static class FieldDimensions {
    public static final Distance fieldWidth = Meters.of(16.54);
    public static final Distance fieldLength = Meters.of(8.07);
  }

  public static class PointsOfInterest {
    public static final Pose2d centerOfHubBlue = new Pose2d(Meters.of(4.625594), Meters.of(4.034536), new Rotation2d(0)); // Point of center of hub, in meters for blue
    public static final Pose2d centerOfHubRed = new Pose2d(Meters.of(11.914406), Meters.of(4.034536), new Rotation2d(0)); // Point of center of hub, in meters for red
  }
}
