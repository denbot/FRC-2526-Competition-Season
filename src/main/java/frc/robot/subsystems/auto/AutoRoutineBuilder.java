package frc.robot.subsystems.auto;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

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
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class AutoRoutineBuilder {

    private ArrayList<Command> commands;
    private Intake intake;
    private Shooter shooter;
    private Indexer indexer;
    private Drive drive;

    public AutoRoutineBuilder(Intake intake, Shooter shooter, Indexer indexer, Drive drive){
        this.intake = intake;
        this.shooter = shooter;
        this.indexer = indexer;
        this.drive = drive;
        this.commands = new ArrayList<>();
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

    public void addExitAlliance(autoOptions exitSide){
        if(exitSide == autoOptions.BORDER_LEFT) addAction(SequentialPathGenerator.getSequentialPath(onTheFlySetpoints.TRENCH_LEFT_ALLIANCE, onTheFlySetpoints.TRENCH_LEFT_NEUTRAL));
        else addAction(SequentialPathGenerator.getSequentialPath(onTheFlySetpoints.TRENCH_RIGHT_ALLIANCE, onTheFlySetpoints.TRENCH_RIGHT_NEUTRAL));
    }
    
    public void addSweep(autoOptions startSide, autoOptions sweepAlignment){
        if(startSide == autoOptions.BORDER_LEFT){
            if(sweepAlignment == autoOptions.SWEEP_EDGE){
                addAction(SequentialPathGenerator.getSequentialPath(onTheFlySetpoints.NEUTRAL_EDGE_LEFT, onTheFlySetpoints.NEUTRAL_EDGE_MID_FROM_LEFT)
                .raceWith(this.intake.runIntake(RotationsPerSecond.of(60))).andThen(this.intake.stopIntake()));
            }
            else{
                addAction(
                    SequentialPathGenerator.getSequentialPath(onTheFlySetpoints.NEUTRAL_CENTER_LEFT, onTheFlySetpoints.NEUTRAL_CENTER_MID_FROM_LEFT)
                    .raceWith(this.intake.runIntake(RotationsPerSecond.of(60))).andThen(this.intake.stopIntake()));
            }
        }
        else{
            if(sweepAlignment == autoOptions.SWEEP_EDGE){
                addAction(SequentialPathGenerator.getSequentialPath(onTheFlySetpoints.NEUTRAL_EDGE_RIGHT, onTheFlySetpoints.NEUTRAL_EDGE_MID_FROM_RIGHT)
                .raceWith(this.intake.runIntake(RotationsPerSecond.of(60))).andThen(this.intake.stopIntake()));
            }
            else{
                addAction(SequentialPathGenerator.getSequentialPath(onTheFlySetpoints.NEUTRAL_CENTER_RIGHT, onTheFlySetpoints.NEUTRAL_CENTER_MID_FROM_RIGHT)
                .raceWith(this.intake.runIntake(RotationsPerSecond.of(60))).andThen(this.intake.stopIntake()));
            }
        }
    }

    public void addReturnAlliance(autoOptions returnSide, autoOptions returnLocation){
        if(returnSide == autoOptions.BORDER_LEFT){
            if(returnLocation == autoOptions.TRENCH){
                addAction(SequentialPathGenerator.getSequentialPath(onTheFlySetpoints.TRENCH_LEFT_NEUTRAL, onTheFlySetpoints.TRENCH_LEFT_ALLIANCE));
            
            }
            else{
                addAction(SequentialPathGenerator.getSequentialPath(onTheFlySetpoints.RAMP_LEFT_NEUTRAL, onTheFlySetpoints.RAMP_LEFT_ALLIANCE));
            }
        }
        else{
            if(returnLocation == autoOptions.TRENCH){
                addAction(SequentialPathGenerator.getSequentialPath(onTheFlySetpoints.TRENCH_RIGHT_NEUTRAL, onTheFlySetpoints.TRENCH_RIGHT_ALLIANCE));
            
            }
            else{
                addAction(SequentialPathGenerator.getSequentialPath(onTheFlySetpoints.RAMP_RIGHT_NEUTRAL, onTheFlySetpoints.RAMP_RIGHT_ALLIANCE));
            }
        }
        
    }

    public void addScoreCommand(){
        // TODO TEMP FUNCTION FOR INDEX AND SHOOT
        addAction(DriveCommands.autoJoystickDriveAtAngle(drive));
        addAction(new ParallelCommandGroup(indexer.runIndexer(), shooter.runKicker(), shooter.runSpinner()).withTimeout(Seconds.of(3)));
        }

    public void addAlignScorePosition(autoOptions scoreLocation){
        switch (scoreLocation) {
            case SHOOT_LEFT:
                addAction(getAutoAlignmentCommand(onTheFlySetpoints.SCORE_LEFT));
                break;

            case SHOOT_RIGHT:
                addAction(getAutoAlignmentCommand(onTheFlySetpoints.SCORE_RIGHT));
                break;
            
            case SHOOT_CENTER:
                addAction(getAutoAlignmentCommand(onTheFlySetpoints.SCORE_CENTER));
                break;
            default:
                break;
        }
    }
    
    public void addFeedCommand(){
        // TODO TEMP FUNCTION FOR INDEX AND SHOOT
        addAction(DriveCommands.autoJoystickDriveAtAngle(drive));
        addAction(new ParallelCommandGroup(indexer.runIndexer(), shooter.runKicker(), shooter.runSpinner()).withTimeout(Seconds.of(3)));
    }

    public void addHumanPlayerCommand(autoOptions endScorePosition){
        addAction(getAutoAlignmentCommand(onTheFlySetpoints.HUMAN_PLAYER));
        addAlignScorePosition(endScorePosition);
        addScoreCommand();
    }

    public void addClimbCommand(autoOptions climbSide){
        // TODO add climb command
        if(climbSide == autoOptions.CLIMB_LEFT){
            addAction(SequentialPathGenerator.getSequentialPath(onTheFlySetpoints.CLIMB_LEFT_SETUP, onTheFlySetpoints.CLIMB_LEFT_FINISH));
            // add climb command
        }
        else {
            addAction(SequentialPathGenerator.getSequentialPath(onTheFlySetpoints.CLIMB_RIGHT_SETUP, onTheFlySetpoints.CLIMB_RIGHT_FINISH));
            // add climb command
        }
    }

    public void testAll(){
        this.addExitAlliance(autoOptions.BORDER_LEFT);
        this.addExitAlliance(autoOptions.BORDER_RIGHT);

        this.addSweep(autoOptions.BORDER_LEFT, autoOptions.SWEEP_CENTER);
        this.addSweep(autoOptions.BORDER_LEFT, autoOptions.SWEEP_EDGE);
        this.addSweep(autoOptions.BORDER_RIGHT, autoOptions.SWEEP_CENTER);
        this.addSweep(autoOptions.BORDER_RIGHT, autoOptions.SWEEP_EDGE);

        this.addFeedCommand();

        this.addReturnAlliance(autoOptions.BORDER_LEFT, autoOptions.TRENCH);
        this.addReturnAlliance(autoOptions.BORDER_LEFT, autoOptions.RAMP);
        this.addReturnAlliance(autoOptions.BORDER_RIGHT, autoOptions.TRENCH);
        this.addReturnAlliance(autoOptions.BORDER_RIGHT, autoOptions.RAMP);

        this.addAlignScorePosition(autoOptions.SHOOT_LEFT);
        this.addAlignScorePosition(autoOptions.SHOOT_CENTER);
        this.addAlignScorePosition(autoOptions.SHOOT_RIGHT);
        
        this.addScoreCommand();

        this.addClimbCommand(autoOptions.CLIMB_LEFT);
        this.addClimbCommand(autoOptions.CLIMB_RIGHT);

        this.addHumanPlayerCommand(autoOptions.SHOOT_LEFT);
        this.addHumanPlayerCommand(autoOptions.SHOOT_CENTER);
        this.addHumanPlayerCommand(autoOptions.SHOOT_RIGHT);
    }

    public void clearRoutine(){
        this.commands.clear();
    }
    
    public void removeLast(){
        this.commands.remove(this.commands.size()-1);
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

    private void addAction(Command command){
        this.commands.add(command);
    }

    public Command getAutoRoutine(){
        SequentialCommandGroup autoRoutine = new SequentialCommandGroup();
        
        ParallelCommandGroup currentGroupCommand = new ParallelCommandGroup();

        for(Command command: this.commands){
            autoRoutine.addCommands(command);
        }

        autoRoutine.addCommands(currentGroupCommand);

        return autoRoutine;
    }
}