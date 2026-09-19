package frc.robot.subsystems.auto;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.ArrayList;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.OperatorConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class AutoRoutineBuilder {

    private ArrayList<Command> commands;
    private ArrayList<String> commandNames;
    public Intake intake;
    public Shooter shooter;
    private Indexer indexer;
    private Drive drive;

    public AutoRoutineBuilder(Intake intake, Shooter shooter, Indexer indexer, Drive drive){
        this.intake = intake;
        this.shooter = shooter;
        this.indexer = indexer;
        this.drive = drive;
        this.commands = new ArrayList<>();
        this.commandNames = new ArrayList<>();
        SmartDashboard.putStringArray("Auto Routine", commandNamesAsStringArray());
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

    public void addExitAllianceTrench(autoOptions exitSide){
        if(exitSide == autoOptions.BORDER_LEFT){
            addAction(
                getAutoAlignmentCommand(TargetPoses.TRENCH_LEFT_NEUTRAL, 1.0)
            .alongWith(this.intake.setIntakeMinLength()),
            "Exit Aliance Left");
        } else {
            addAction(
                getAutoAlignmentCommand(TargetPoses.TRENCH_RIGHT_NEUTRAL, 1.0)
                .alongWith(this.intake.setIntakeMinLength()),
                "Exit Aliance Right");
        }
    }
    public void addExitAllianceRamp(autoOptions exitSide){
        if(exitSide == autoOptions.BORDER_LEFT){
            addAction(
                getAutoAlignmentCommand(TargetPoses.RAMP_LEFT_NEUTRAL, 1.0)
                .alongWith(this.intake.setIntakeMinLength()),
                "Exit Aliance Left");
        } else {
            addAction(
                getAutoAlignmentCommand(TargetPoses.RAMP_RIGHT_NEUTRAL, 1.0)
                .alongWith(this.intake.setIntakeMinLength()),
                "Exit Aliance Right");
        }
    }
    
    public void addSweep(autoOptions startSide, autoOptions sweepAlignment){
        if(startSide == autoOptions.BORDER_LEFT){
            if(sweepAlignment == autoOptions.SWEEP_EDGE){
                addAction(
                    SequentialPathGenerator.getSequentialPath(
                        new double[]{4.0, 4.0}, 
                        new double[]{4.0, 4.0}, 
                        TargetPoses.NEUTRAL_EDGE_LEFT, 
                        TargetPoses.NEUTRAL_EDGE_MID_FROM_LEFT)
                    .raceWith(getIntakeCommand()),
                    "Sweep Edge Left");
            }
            else{
                addAction(
                    SequentialPathGenerator.getSequentialPath(
                        new double[]{4.0, 4.0}, 
                        new double[]{4.0, 4.0}, 
                        TargetPoses.NEUTRAL_CENTER_LEFT, 
                        TargetPoses.NEUTRAL_CENTER_MID_FROM_LEFT)
                    .raceWith(getIntakeCommand()),
                    "Sweep Center Left");
            }
        }
        else{
            if(sweepAlignment == autoOptions.SWEEP_EDGE){
                addAction(
                    SequentialPathGenerator.getSequentialPath(
                        new double[]{4.0, 4.0}, 
                        new double[]{4.0, 4.0}, 
                        TargetPoses.NEUTRAL_EDGE_RIGHT, 
                        TargetPoses.NEUTRAL_EDGE_MID_FROM_RIGHT)
                    .raceWith(getIntakeCommand()),
                    "Sweep Edge Right");
            }
            else{
                addAction(
                    SequentialPathGenerator.getSequentialPath(
                        new double[]{4.0, 4.0}, 
                        new double[]{4.0, 4.0}, 
                        TargetPoses.NEUTRAL_CENTER_RIGHT, 
                        TargetPoses.NEUTRAL_CENTER_MID_FROM_RIGHT)
                    .raceWith(getIntakeCommand()),
                    "Sweep Center Right");
            }
        }
    }

    public void addReturnAlliance(autoOptions returnSide, autoOptions returnLocation){
        if(returnSide == autoOptions.BORDER_LEFT){
            if(returnLocation == autoOptions.TRENCH){
                addAction(
                    SequentialPathGenerator.getSequentialPath(
                        new double[]{2.0, 0.0}, 
                        TargetPoses.TRENCH_LEFT_NEUTRAL, 
                        TargetPoses.TRENCH_LEFT_ALLIANCE)
                    .alongWith()
                    , "Return Left Through Trench");
            
            }
            else{
                addAction(
                    SequentialPathGenerator.getSequentialPath(
                        new double[]{2.0, 0.0}, 
                        TargetPoses.RAMP_LEFT_NEUTRAL, 
                        TargetPoses.RAMP_LEFT_ALLIANCE)
                    .alongWith(getChurnCommand())
                    , "Return Left Through Ramp");
            }
        }
        else{
            if(returnLocation == autoOptions.TRENCH){
                addAction(
                    SequentialPathGenerator.getSequentialPath(
                        new double[]{4.0, 0.0}, 
                        TargetPoses.TRENCH_RIGHT_NEUTRAL, 
                        TargetPoses.TRENCH_RIGHT_ALLIANCE)
                    .alongWith(getChurnCommand())
                    , "Return Right Through Trench");
            
            }
            else{
                addAction(
                    SequentialPathGenerator.getSequentialPath(
                        new double[]{4.0, 0.0}, 
                        TargetPoses.RAMP_RIGHT_NEUTRAL, 
                        TargetPoses.RAMP_RIGHT_ALLIANCE)
                    .alongWith(getChurnCommand())
                    , "Return Right Through Ramp");
            }
        }
        
    }

    public void addShootCommand(){
        addAction(getShootCommand(), "Shoot");
    }

    public void addAlignScorePosition(autoOptions scoreLocation){
        switch (scoreLocation) {
            case SHOOT_LEFT:
                addAction(getAutoAlignmentCommand(TargetPoses.SCORE_LEFT,1.0)
                    .alongWith(getChurnCommand())
                    , "Align Shoot Left");
                break;

            case SHOOT_RIGHT:
                addAction(getAutoAlignmentCommand(TargetPoses.SCORE_RIGHT,1.0)
                    .alongWith(getChurnCommand())
                    , "Align Shoot Right");
                break;
            
            case SHOOT_CENTER:
                addAction(getAutoAlignmentCommand(TargetPoses.SCORE_CENTER,1.0)
                    .alongWith(getChurnCommand())
                    , "Align Shoot Center");
                break;
            default:
                break;
        }
    }
    
    public void addHumanPlayerCommand(autoOptions endScorePosition){
      
        addAction(
            getAutoAlignmentCommand(TargetPoses.HUMAN_PLAYER,0.0)
            .alongWith(intake.setIntakeMaxLength())
            , "Align To Human Player");
        addAction(Commands.waitSeconds(2), "Wait For HP");
        addAlignScorePosition(endScorePosition);
        addShootCommand();
    }

    public String[] commandNamesAsStringArray(){
        String[] sArray = new String[commandNames.size()];
        commandNames.toArray(sArray);
        return sArray;
    }

    public void clearRoutine(){
        this.commands.clear();
        this.commandNames.clear();
        SmartDashboard.putStringArray("Auto Routine", commandNamesAsStringArray());
    }
    
    public void removeLast(){
        if(this.commands.size() <= 0) return;
        this.commands.remove(this.commands.size()-1);
        this.commandNames.remove(this.commandNames.size()-1);
        SmartDashboard.putStringArray("Auto Routine", commandNamesAsStringArray());
    }

    public Command getAutoAlignmentCommand(TargetPoses setpoint, double endSpeed){
        Pose2d targetPose;
        if(RobotContainer.isBlue()) targetPose = setpoint.blueAlignmentPose;
        else targetPose = setpoint.redAlignmentPose;

        return AutoBuilder.pathfindToPose(
            targetPose,
            OperatorConstants.pathfindingConstraints,
            endSpeed);
    }

    public Command getIntakeCommand() {
        return this.intake.setIntakeMaxLength()
                    .alongWith(this.intake.runIntake(RotationsPerSecond.of(80)))
                    .alongWith(this.indexer.runIndexer());
    }

    public Command getChurnCommand() {
        return this.intake.stopIntake()
                    .alongWith(this.indexer.reverseIndexer())
                    .alongWith(this.shooter.reverseKicker());
    }

    public Command getShootCommand() {
        return intake.setIntakeMaxLength() // extend intake for maximum storage space
            // Run the spinner up to speed until it is at speed
            .alongWith(DriveCommands.autoJoystickDriveAtAngle(drive)) // Auto aim at the hub
            .andThen(shooter.runSpinnerAdaptive(drive))
            .until(() -> Math.abs(shooter.getSpinnerClosedLoopError()) < 1 && shooter.getLeftSpinnerVelocity().magnitude() > 30) // Run only the spin up and auto aim commands until the spinner is at speed
            .andThen(
                // Continue running spinner at speed
                shooter.runSpinnerAdaptive(drive)
                .alongWith(DriveCommands.autoJoystickDriveAtAngle(drive)).withTimeout(4) // Auto aim at the hub
                // Run indexer and kicker to feed shooter with fuel
                .alongWith(indexer.runIndexer()).withTimeout(4)
                .alongWith(shooter.runKicker()).withTimeout(4)
                // wait 2 seconds to fire majority of fuel, then retract intake to shove extra balls into the system
                .alongWith(
                    Commands.waitSeconds(2)
                    .andThen(intake.setIntakeMinLength())).withTimeout(4))
            .andThen(
                indexer.stopIndexer()
                .alongWith(shooter.stopKicker())
                .alongWith(intake.setIntakeMaxLength()));
    }

    public void addAction(Command command, String commandName){
        this.commands.add(command);
        this.commandNames.add(commandName);
        SmartDashboard.putStringArray("Auto Routine", commandNamesAsStringArray());
    }

    public Command getAutoRoutine(){
        SequentialCommandGroup autoRoutine = new SequentialCommandGroup();
        
        for(Command command: this.commands){
            autoRoutine.addCommands(command);
        }

        return autoRoutine;
    }
}