package frc.robot.subsystems.auto;

import java.util.ArrayList;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoRoutineBuilder {
    private static ArrayList<Command> autoRoutine = new ArrayList<>();
    private static ArrayList<onTheFlySetpoints> currentMovementList = new ArrayList<>();

    public enum autoOptions{
        BORDER_LEFT,
        BORDER_RIGHT,
        TRENCH,
        RAMP,
        SWEEP_EDGE,
        SWEEP_CENTER,
        CLIMB_LEFT,
        CLIMB_RIGHT,
        SHOOT_LEFT,
        SHOOT_CENTER,
        SHOOT_RIGHT;
    }

    public static void addExitAlliance(autoOptions exitSide, autoOptions exitLocation){
        if(exitSide == autoOptions.BORDER_LEFT){
            if(exitLocation == autoOptions.TRENCH){
                currentMovementList.add(onTheFlySetpoints.TRENCH_LEFT_ALLIANCE);
                currentMovementList.add(onTheFlySetpoints.TRENCH_LEFT_NEUTRAL);
            
            }
            else{
                currentMovementList.add(onTheFlySetpoints.RAMP_LEFT_ALLIANCE);
                currentMovementList.add(onTheFlySetpoints.RAMP_LEFT_NEUTRAL);
            }
        }
        else{
            if(exitLocation == autoOptions.TRENCH){
                currentMovementList.add(onTheFlySetpoints.TRENCH_RIGHT_ALLIANCE);
                currentMovementList.add(onTheFlySetpoints.TRENCH_RIGHT_NEUTRAL);
            
            }
            else{
                currentMovementList.add(onTheFlySetpoints.RAMP_RIGHT_ALLIANCE);
                currentMovementList.add(onTheFlySetpoints.RAMP_RIGHT_NEUTRAL);
            }
        }
    }
    
    public static void addSweep(autoOptions startSide, autoOptions sweepAlignment){
        //addMovementList();
        //TODO add run intake command
        if(startSide == autoOptions.BORDER_LEFT){
            if(sweepAlignment == autoOptions.SWEEP_EDGE){
                currentMovementList.add(onTheFlySetpoints.NEUTRAL_EDGE_LEFT);
                currentMovementList.add(onTheFlySetpoints.NEUTRAL_EDGE_MID);
            
            }
            else{
                currentMovementList.add(onTheFlySetpoints.NEUTRAL_CENTER_LEFT);
                currentMovementList.add(onTheFlySetpoints.NEUTRAL_CENTER_MID);
            }
        }
        else{
            if(sweepAlignment == autoOptions.SWEEP_EDGE){
                currentMovementList.add(onTheFlySetpoints.NEUTRAL_EDGE_RIGHT);
                currentMovementList.add(onTheFlySetpoints.NEUTRAL_EDGE_MID);
            
            }
            else{
                currentMovementList.add(onTheFlySetpoints.NEUTRAL_CENTER_RIGHT);
                currentMovementList.add(onTheFlySetpoints.NEUTRAL_CENTER_MID);
            }
        }
    }

    public static void addReturnAlliance(autoOptions returnSide, autoOptions returnLocation){
        if(returnSide == autoOptions.BORDER_LEFT){
            if(returnLocation == autoOptions.TRENCH){
                currentMovementList.add(onTheFlySetpoints.TRENCH_LEFT_NEUTRAL);
                currentMovementList.add(onTheFlySetpoints.TRENCH_LEFT_ALLIANCE);
            
            }
            else{
                currentMovementList.add(onTheFlySetpoints.RAMP_LEFT_NEUTRAL);
                currentMovementList.add(onTheFlySetpoints.RAMP_LEFT_ALLIANCE);
            }
        }
        else{
            if(returnLocation == autoOptions.TRENCH){
                currentMovementList.add(onTheFlySetpoints.TRENCH_RIGHT_NEUTRAL);
                currentMovementList.add(onTheFlySetpoints.TRENCH_RIGHT_ALLIANCE);
            
            }
            else{
                currentMovementList.add(onTheFlySetpoints.RAMP_RIGHT_NEUTRAL);
                currentMovementList.add(onTheFlySetpoints.RAMP_RIGHT_ALLIANCE);
            }
        }
        
    }

    public static void addScoreCommand(autoOptions scoreLocation){
        addMovementList();
        switch (scoreLocation) {
            case SHOOT_LEFT:
                autoRoutine.add(
                    getAutoAlignmentCommand(onTheFlySetpoints.SCORE_LEFT));
                break;

            case SHOOT_RIGHT:
                autoRoutine.add(
                    getAutoAlignmentCommand(onTheFlySetpoints.SCORE_RIGHT));
                break;
            
            case SHOOT_CENTER:
                autoRoutine.add(
                    getAutoAlignmentCommand(onTheFlySetpoints.SCORE_CENTER));
                break;
            default:
                break;
        }
        //TODO add aim and shoot commands
        //autoRoutine.add(AimCommand)
        //autoRoutine.add(ShootCommand)
    }
    
    public static void addFeedCommand(){
        addMovementList();
        //TODO add aim and shoot commands
        //autoRoutine.add(AimCommand)
        //autoRoutine.add(ShootCommand)
    }

    public static void addHumanPlayerCommand(autoOptions endScorePosition){
        addMovementList();
        autoRoutine.add(
            getAutoAlignmentCommand(onTheFlySetpoints.HUMAN_PLAYER));
        addScoreCommand(endScorePosition);
    }

    public static void addClimbCommand(autoOptions climbSide){
        addMovementList();
        // TODO add climb command
        if(climbSide == autoOptions.CLIMB_LEFT){
            autoRoutine.add(getAutoAlignmentCommand(onTheFlySetpoints.CLIMB_LEFT_SETUP));
            autoRoutine.add(getAutoAlignmentCommand(onTheFlySetpoints.CLIMB_LEFT_FINISH));
                // add climb command
        }
        else {
            autoRoutine.add(getAutoAlignmentCommand(onTheFlySetpoints.CLIMB_RIGHT_SETUP));
            autoRoutine.add(getAutoAlignmentCommand(onTheFlySetpoints.CLIMB_RIGHT_FINISH));
            // add climb command
        }
    }

    public static void clearRoutine(){
        autoRoutine.clear();
    }
    
    public static void removeLast(){
        autoRoutine.remove(autoRoutine.size()-1);
    }

    private static Command getAutoAlignmentCommand(onTheFlySetpoints setpoint){
        Optional<Alliance> alliance = DriverStation.getAlliance();
        Pose2d targetPose;
        if(alliance.isPresent() && alliance.get() == Alliance.Red) targetPose = setpoint.redAlignmentPose;
        else targetPose = setpoint.blueAlignmentPose;

        return AutoBuilder.pathfindToPose(
            targetPose,
            new PathConstraints(6.0, 6.0, Units.degreesToRadians(540), Units.degreesToRadians(720)));
    }

    private static void addMovementList(){
        if(currentMovementList.size() == 0) return;
        onTheFlySetpoints[] setpoints = new onTheFlySetpoints[currentMovementList.size()];
        currentMovementList.toArray(setpoints);
        autoRoutine.add(ConcurrentPathGenerator.getConcurrentPath(setpoints));
        currentMovementList.clear();
    }

    public static Command getAutoRoutine(){
        Command[] commands = new Command[autoRoutine.size()];
        autoRoutine.toArray(commands);
        return new SequentialCommandGroup(commands);
    }
}