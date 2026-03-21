package frc.robot.subsystems.auto;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drive;

public class AutoCommandHelper {
    private static SequentialCommandGroup autoRoutine = new SequentialCommandGroup();
    private static List<Pose2d> currentPathSetpoints;
    private static PathConstraints currentConstraints = new PathConstraints(6, 6, 6, 6);
    private static List<Command> subsystemCommands;
    private static List<Integer> subsystemTargetPoseIndicies;
    
    public static void addSetpoint(Pose2d setpoint){
        currentPathSetpoints.add(setpoint);
    }

    public static void endPath(double endVelocityMPS){
        PathPlannerPath path = new PathPlannerPath(
            PathPlannerPath.waypointsFromPoses(currentPathSetpoints), // Convert Poses into pathplanner waypoints 
            currentConstraints, // use the current constraints defined by the setMax___Velocity functions
            null, 
            new GoalEndState(endVelocityMPS, currentPathSetpoints.get(currentPathSetpoints.size()-1).getRotation())); 

        Command pathCommand = AutoBuilder.pathfindThenFollowPath(path, currentConstraints);
        ParallelCommandGroup totalCommand = new ParallelCommandGroup(pathCommand);

        // get a trajecoty object for the path so that we can refference the current target pose when the path is running
        PathPlannerTrajectory trajectory = new PathPlannerTrajectory(path, new ChassisSpeeds(), new Rotation2d(), Drive.PP_CONFIG);
        PathPlannerTrajectoryState targetState = trajectory.sample(2.0); // sample target pose

        // for all commands added throughout the definition of this path
        for(int i = 0; i < subsystemCommands.size(); i++){
            int index = subsystemTargetPoseIndicies.get(i); // get the pose at which the command was added

            // parallel the command with the motion path but only run the command if the current target pose is the pose after the command was added
            // IE: added command after setpoint 2, it triggers once the path is trying to now go to sepoint 3 or the path just reached setpoint 2
            totalCommand.addCommands(
                    subsystemCommands.get(i)
                        .onlyIf(()->
                            targetState.pose.equals(currentPathSetpoints.get(index+index==currentPathSetpoints.size()?0:1))));
        }

        // clear temp lists
        currentPathSetpoints.clear();
        subsystemCommands.clear();
        subsystemTargetPoseIndicies.clear();

        autoRoutine.addCommands(pathCommand);
    }

    public static void addSubsystemAction(Command subsystemAction){
        subsystemCommands.add(subsystemAction);
        subsystemTargetPoseIndicies.add(currentPathSetpoints.size()-1);
    }

    public static void addPause(Time duration){
        endPath(0);
        autoRoutine.addCommands(Commands.waitTime(duration));
    }

    public static void setMaxLinearVelocity(LinearVelocity maxVelocity){
        currentConstraints = new PathConstraints(maxVelocity.baseUnitMagnitude(), 6, currentConstraints.maxAngularVelocityRadPerSec(), 6);
        endPath(maxVelocity.baseUnitMagnitude());
    }

    public static void setMaxAngularVelocity(AngularVelocity maxVelocity){
        currentConstraints = new PathConstraints(currentConstraints.maxVelocityMPS(), 6, maxVelocity.baseUnitMagnitude(), 6);
        endPath(currentConstraints.maxVelocityMPS());
    }

    public static Command getRoutine(){
        return autoRoutine;
    }

}
