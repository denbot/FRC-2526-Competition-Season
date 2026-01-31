package frc.robot.commands;
import frc.robot.Constants.PointsOfInterest;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class AutoAimCommandHelper {
    public AutoAimCommandHelper() {}

    public Rotation2d findAngleForShooting(Pose2d pose) {
        Distance positionX = pose.getMeasureX();
        Distance positionY = pose.getMeasureY();

        Distance deltaX;
        Distance deltaY;

        if(isBlue()) {
            if (positionX.minus(PointsOfInterest.centerOfHubBlue.getMeasureX()).in(Meters) > 0) { // If we are not in our zone
                if (positionY.minus(PointsOfInterest.centerOfHubBlue.getMeasureY()).in(Meters) > 0) { // If we are in the north half of the field
                    deltaX = PointsOfInterest.cornerNW.getMeasureX().minus(positionX);
                    deltaY = PointsOfInterest.cornerNW.getMeasureY().minus(positionY);
                } else { // If we are in the south half of the field
                    deltaX = PointsOfInterest.cornerSW.getMeasureX().minus(positionX);
                    deltaY = PointsOfInterest.cornerSW.getMeasureY().minus(positionY);
                }
            } else { // If we are in our zone (able to shoot at hub)
                deltaX = PointsOfInterest.centerOfHubBlue.getMeasureX().minus(positionX);
                deltaY = PointsOfInterest.centerOfHubBlue.getMeasureY().minus(positionY);
            }
        } else {
            if (positionX.minus(PointsOfInterest.centerOfHubRed.getMeasureX()).in(Meters) < 0) { // If we are not in our zone
                if (positionY.minus(PointsOfInterest.centerOfHubRed.getMeasureY()).in(Meters) > 0) { // If we are in the north half of the field
                    deltaX = PointsOfInterest.cornerNE.getMeasureX().minus(positionX);
                    deltaY = PointsOfInterest.cornerNE.getMeasureY().minus(positionY);
                } else { // If we are in the south half of the field
                    deltaX = PointsOfInterest.cornerSE.getMeasureX().minus(positionX);
                    deltaY = PointsOfInterest.cornerSE.getMeasureY().minus(positionY);
                }
            } else { // If we are in our zone (able to shoot at hub)
                deltaX = PointsOfInterest.centerOfHubRed.getMeasureX().minus(positionX);
                deltaY = PointsOfInterest.centerOfHubRed.getMeasureY().minus(positionY);
            }
        }
            
        Rotation2d rotation = new Rotation2d(Math.atan(deltaY.baseUnitMagnitude() / deltaX.baseUnitMagnitude()));
        return rotation;
    }

    private boolean isBlue() {
        return DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Blue;
    }
}
