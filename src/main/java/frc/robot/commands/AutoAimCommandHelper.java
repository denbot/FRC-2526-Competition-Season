package frc.robot.commands;
import frc.robot.Constants.PointsOfInterest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;

public class AutoAimCommandHelper {
    private AutoAimCommandHelper() {}

    public Rotation2d findAngleForShooting(Pose2d pose) {
        Distance positionX = pose.getMeasureX();
        Distance positionY = pose.getMeasureY();
        Distance deltaX = PointsOfInterest.centerOfHub.getMeasureX().minus(positionX);
        Distance deltaY = PointsOfInterest.centerOfHub.getMeasureY().minus(positionY);
        Rotation2d rotation = new Rotation2d(Math.atan(deltaY.baseUnitMagnitude() / deltaX.baseUnitMagnitude()));
        return rotation;
    }
}
