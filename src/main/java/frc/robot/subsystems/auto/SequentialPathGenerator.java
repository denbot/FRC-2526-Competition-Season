package frc.robot.subsystems.auto;

import static edu.wpi.first.units.Units.Degree;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.OperatorConstants;

public class SequentialPathGenerator {
    public static Command getSequentialPath(boolean isBlue, double[] maxSpeeds, onTheFlySetpoints[] setpoints, Double[] angles){
        SequentialCommandGroup finalPath = new SequentialCommandGroup();

        for(int i = 0; i < setpoints.length; i++){
            Pose2d targetPose;
            if(!isBlue) targetPose = setpoints[i].redAlignmentPose;
            else targetPose = setpoints[i].blueAlignmentPose;

            finalPath.addCommands(
                AutoBuilder.pathfindToPose(new Pose2d(targetPose.getX(), targetPose.getY(), new Rotation2d(Degree.of(angles[i]))),
                new PathConstraints(maxSpeeds[i], maxSpeeds[i], Units.degreesToRadians(540), Units.degreesToRadians(720))));
        }
        return finalPath;
    }
    public static Command getSequentialPath(boolean isBlue, onTheFlySetpoints... setpoints){
        SequentialCommandGroup finalPath = new SequentialCommandGroup();

        for(int i = 0; i < setpoints.length; i++){
            Pose2d targetPose;
            if(!isBlue) targetPose = setpoints[i].redAlignmentPose;
            else targetPose = setpoints[i].blueAlignmentPose;

            finalPath.addCommands(
                AutoBuilder.pathfindToPose(targetPose,
                OperatorConstants.pathfindingConstraints));
        }
        return finalPath;
    }
    
    public static Command getSequentialPath(boolean isBlue, double[] maxSpeeds, onTheFlySetpoints... setpoints){
        SequentialCommandGroup finalPath = new SequentialCommandGroup();

        for(int i = 0; i < setpoints.length; i++){
            Pose2d targetPose;
            if(!isBlue) targetPose = setpoints[i].redAlignmentPose;
            else targetPose = setpoints[i].blueAlignmentPose;

            finalPath.addCommands(
                AutoBuilder.pathfindToPose(targetPose,
                new PathConstraints(maxSpeeds[i], maxSpeeds[i], Units.degreesToRadians(540), Units.degreesToRadians(720))));
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
