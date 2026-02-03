package frc.robot.subsystems.auto;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.ArrayList;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoAimCommandHelper;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class AutoRoutineBuilder {

    private ArrayList<autoAction> actions;
    private Intake intake;
    private Shooter shooter;
    private Indexer indexer;
    private Drive drive;
    AutoAimCommandHelper autoAimCommandHelper;

    public AutoRoutineBuilder(Intake intake, Shooter shooter, Indexer indexer, Drive drive, AutoAimCommandHelper autoAimCommandHelper){
        this.intake = intake;
        this.shooter = shooter;
        this.indexer = indexer;
        this.drive = drive;
        this.autoAimCommandHelper = autoAimCommandHelper;
        this.actions = new ArrayList<>();
    }

    private enum actionTypes{
        ALIGN,
        SHOOT,
        INTAKE,
        INDEX;
    }

    private class autoAction{
        public actionTypes type;
        public Command command;
        public autoAction(actionTypes type, Command command){
            this.type = type;
            this.command = command;
        }
    }

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

    public void addExitAlliance(autoOptions exitSide, autoOptions exitLocation){
        if(exitSide == autoOptions.BORDER_LEFT){
            if(exitLocation == autoOptions.TRENCH){
                addAction(actionTypes.ALIGN, ConcurrentPathGenerator.getConcurrentPath(onTheFlySetpoints.TRENCH_LEFT_ALLIANCE, onTheFlySetpoints.TRENCH_LEFT_NEUTRAL));
            }
            else{
                addAction(actionTypes.ALIGN, ConcurrentPathGenerator.getConcurrentPath(onTheFlySetpoints.RAMP_LEFT_ALLIANCE, onTheFlySetpoints.RAMP_LEFT_NEUTRAL));
            }
        }
        else{
            if(exitLocation == autoOptions.TRENCH){
                addAction(actionTypes.ALIGN, ConcurrentPathGenerator.getConcurrentPath(onTheFlySetpoints.TRENCH_RIGHT_ALLIANCE, onTheFlySetpoints.TRENCH_RIGHT_NEUTRAL));
            
            }
            else{
                addAction(actionTypes.ALIGN, ConcurrentPathGenerator.getConcurrentPath(onTheFlySetpoints.RAMP_RIGHT_ALLIANCE, onTheFlySetpoints.RAMP_RIGHT_NEUTRAL));
            }
        }
    }
    
    public void addSweep(autoOptions startSide, autoOptions sweepAlignment){
        addAction(actionTypes.INTAKE, this.intake.runIntake(RotationsPerSecond.of(60)));
        if(startSide == autoOptions.BORDER_LEFT){
            if(sweepAlignment == autoOptions.SWEEP_EDGE){
                addAction(actionTypes.ALIGN, ConcurrentPathGenerator.getConcurrentPath(onTheFlySetpoints.NEUTRAL_EDGE_LEFT, onTheFlySetpoints.NEUTRAL_EDGE_MID));
            }
            else{
                addAction(actionTypes.ALIGN, ConcurrentPathGenerator.getConcurrentPath(onTheFlySetpoints.NEUTRAL_CENTER_LEFT, onTheFlySetpoints.NEUTRAL_CENTER_MID));
            }
        }
        else{
            if(sweepAlignment == autoOptions.SWEEP_EDGE){
                addAction(actionTypes.ALIGN, ConcurrentPathGenerator.getConcurrentPath(onTheFlySetpoints.NEUTRAL_EDGE_RIGHT, onTheFlySetpoints.NEUTRAL_EDGE_MID));
            
            }
            else{
                addAction(actionTypes.ALIGN, ConcurrentPathGenerator.getConcurrentPath(onTheFlySetpoints.NEUTRAL_CENTER_RIGHT, onTheFlySetpoints.NEUTRAL_CENTER_MID));
            }
        }
    }

    public void addReturnAlliance(autoOptions returnSide, autoOptions returnLocation){
        if(returnSide == autoOptions.BORDER_LEFT){
            if(returnLocation == autoOptions.TRENCH){
                addAction(actionTypes.ALIGN, ConcurrentPathGenerator.getConcurrentPath(onTheFlySetpoints.TRENCH_LEFT_NEUTRAL, onTheFlySetpoints.TRENCH_LEFT_ALLIANCE));
            
            }
            else{
                addAction(actionTypes.ALIGN, ConcurrentPathGenerator.getConcurrentPath(onTheFlySetpoints.RAMP_LEFT_NEUTRAL, onTheFlySetpoints.RAMP_LEFT_ALLIANCE));
            }
        }
        else{
            if(returnLocation == autoOptions.TRENCH){
                addAction(actionTypes.ALIGN, ConcurrentPathGenerator.getConcurrentPath(onTheFlySetpoints.TRENCH_RIGHT_NEUTRAL, onTheFlySetpoints.TRENCH_RIGHT_ALLIANCE));
            
            }
            else{
                addAction(actionTypes.ALIGN, ConcurrentPathGenerator.getConcurrentPath(onTheFlySetpoints.RAMP_RIGHT_NEUTRAL, onTheFlySetpoints.RAMP_RIGHT_ALLIANCE));
            }
        }
        
    }

    public void addScoreCommand(autoOptions scoreLocation){
        switch (scoreLocation) {
            case SHOOT_LEFT:
                addAction(actionTypes.ALIGN, getAutoAlignmentCommand(onTheFlySetpoints.SCORE_LEFT));
                break;

            case SHOOT_RIGHT:
                addAction(actionTypes.ALIGN, getAutoAlignmentCommand(onTheFlySetpoints.SCORE_RIGHT));
                break;
            
            case SHOOT_CENTER:
                addAction(actionTypes.ALIGN, getAutoAlignmentCommand(onTheFlySetpoints.SCORE_CENTER));
                break;
            default:
                break;
        }
        // TODO TEMP FUNCTION FOR INDEX AND SHOOT
        addAction(actionTypes.ALIGN, DriveCommands.joystickDriveAtAngle(
                drive,
                () -> 0,
                () -> 0,
                () -> autoAimCommandHelper.findAngleForShooting(drive.getPose()).times(1.0)));
        addAction(actionTypes.INDEX, indexer.runIndexer());
        addAction(actionTypes.INDEX, shooter.runKicker());
        addAction(actionTypes.SHOOT, shooter.runSpinner());
        }
    
    public void addFeedCommand(){
        // TODO TEMP FUNCTION FOR INDEX AND SHOOT
        addAction(actionTypes.ALIGN, DriveCommands.joystickDriveAtAngle(
                drive,
                () -> 0,
                () -> 0,
                () -> autoAimCommandHelper.findAngleForShooting(drive.getPose()).times(1.0)));
        addAction(actionTypes.INDEX, indexer.runIndexer());
        addAction(actionTypes.INDEX, shooter.runKicker());
        addAction(actionTypes.SHOOT, shooter.runSpinner());
    }

    public void addHumanPlayerCommand(autoOptions endScorePosition){
        addAction(actionTypes.ALIGN, getAutoAlignmentCommand(onTheFlySetpoints.HUMAN_PLAYER));
        addScoreCommand(endScorePosition);
    }

    public void addClimbCommand(autoOptions climbSide){
        // TODO add climb command
        if(climbSide == autoOptions.CLIMB_LEFT){
            addAction(actionTypes.ALIGN, ConcurrentPathGenerator.getConcurrentPath(onTheFlySetpoints.CLIMB_LEFT_SETUP, onTheFlySetpoints.CLIMB_LEFT_FINISH));
            // add climb command
        }
        else {
            addAction(actionTypes.ALIGN, ConcurrentPathGenerator.getConcurrentPath(onTheFlySetpoints.CLIMB_RIGHT_SETUP, onTheFlySetpoints.CLIMB_RIGHT_FINISH));
            // add climb command
        }
    }

    public void clearRoutine(){
        this.actions.clear();
    }
    
    public void removeLast(){
        this.actions.remove(this.actions.size()-1);
    }

    private Command getAutoAlignmentCommand(onTheFlySetpoints setpoint){
        Optional<Alliance> alliance = DriverStation.getAlliance();
        Pose2d targetPose;
        if(alliance.isPresent() && alliance.get() == Alliance.Red) targetPose = setpoint.redAlignmentPose;
        else targetPose = setpoint.blueAlignmentPose;

        return AutoBuilder.pathfindToPose(
            targetPose,
            new PathConstraints(6.0, 6.0, Units.degreesToRadians(540), Units.degreesToRadians(720)));
    }

    private void addAction(actionTypes type, Command command){
        this.actions.add(new autoAction(type, command));
    }

    public Command getAutoRoutine(){
        SequentialCommandGroup autoRoutine = new SequentialCommandGroup();
        
        ParallelCommandGroup currentGroupCommand = new ParallelCommandGroup();

        for(autoAction action: this.actions){
            if(action.type == actionTypes.ALIGN){
                autoRoutine.addCommands(currentGroupCommand);
                currentGroupCommand = new ParallelCommandGroup(action.command);
            }
            else currentGroupCommand.addCommands(action.command);
        }

        autoRoutine.addCommands(currentGroupCommand);

        return autoRoutine;
    }
}