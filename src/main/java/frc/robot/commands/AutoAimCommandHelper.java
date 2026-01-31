package frc.robot.commands;
import frc.robot.Constants.PointsOfInterest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class AutoAimCommandHelper {
    private AutoAimCommandHelper() {}

    public Rotation2d findAngleForShooting(Pose2d pose) {
        Distance positionX = pose.getMeasureX();
        Distance positionY = pose.getMeasureY();

        Distance deltaX;
        Distance deltaY;

        if (isBlue()) {
            deltaX = PointsOfInterest.centerOfHubBlue.getMeasureX().minus(positionX);
            deltaY = PointsOfInterest.centerOfHubBlue.getMeasureY().minus(positionY);
        } else {
            deltaX = PointsOfInterest.centerOfHubRed.getMeasureX().minus(positionX);
            deltaY = PointsOfInterest.centerOfHubRed.getMeasureY().minus(positionY);
        }
        
        Rotation2d rotation = new Rotation2d(Math.atan(deltaY.baseUnitMagnitude() / deltaX.baseUnitMagnitude()));
        return rotation;
    }

    private boolean isBlue() {
        return DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Blue;
    }
}
