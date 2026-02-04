package frc.robot.subsystems.auto;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.OperatorConstants;

public class SequentialPathGenerator {
    public static Command getSequentialPath(onTheFlySetpoints[] setpoints, Double[] angles){
        SequentialCommandGroup finalPath = new SequentialCommandGroup();
        Optional<Alliance> alliance = DriverStation.getAlliance();

        for(int i = 0; i < setpoints.length; i++){
            Pose2d targetPose;
            if(alliance.isPresent() && alliance.get() == Alliance.Red) targetPose = setpoints[i].redAlignmentPose;
            else targetPose = setpoints[i].blueAlignmentPose;

            finalPath.addCommands(
                AutoBuilder.pathfindToPose(new Pose2d(targetPose.getX(), targetPose.getY(), new Rotation2d(angles[i])),
                OperatorConstants.pathfindingConstraints));
        }
        return finalPath;
    }
    public static Command getSequentialPath(onTheFlySetpoints... setpoints){
        SequentialCommandGroup finalPath = new SequentialCommandGroup();
        Optional<Alliance> alliance = DriverStation.getAlliance();

        for(int i = 0; i < setpoints.length; i++){
            Pose2d targetPose;
            if(alliance.isPresent() && alliance.get() == Alliance.Red) targetPose = setpoints[i].redAlignmentPose;
            else targetPose = setpoints[i].blueAlignmentPose;

            finalPath.addCommands(
                AutoBuilder.pathfindToPose(targetPose,
                OperatorConstants.pathfindingConstraints));
        }
        return finalPath;
    }
    
    /* Possible chaining of auto alignments using waypoints
    public static Command getConcurrentPath(onTheFlySetpoints[] setpoints){
        Optional<Alliance> alliance = DriverStation.getAlliance();
        Pose2d[] poses = new Pose2d[setpoints.length];

        for(int i = 0; i < setpoints.length; i++){
            Pose2d targetPose;
            if(alliance.isPresent() && alliance.get() == Alliance.Red) targetPose = setpoints[i].redAlignmentPose;
            else targetPose = setpoints[i].blueAlignmentPose;
            poses[i] = targetPose;
        }


        if(poses.length == 1) return AutoBuilder.pathfindToPose(poses[0], pathConstraints);

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(poses);

        PathPlannerPath path = new PathPlannerPath(waypoints, pathConstraints, null, new GoalEndState(0, new Rotation2d()));
                
        return AutoBuilder.pathfindThenFollowPath(path, pathConstraints);
    }
    */
}
