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
import frc.robot.Constants.PointsOfInterest;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class AutoRoutineBuilder {

    private ArrayList<Command> commands;
    private ArrayList<String> commandNames;
    private Intake intake;
    public Shooter shooter;
    private Indexer indexer;
    private Drive drive;
    private boolean isBlue = true;

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

    public void setIsBlue(boolean isBlue){
        this.isBlue = isBlue;
    }

    public void addExitAlliance(autoOptions exitSide){
        if(exitSide == autoOptions.BORDER_LEFT) addAction(getAutoAlignmentCommand(isBlue, onTheFlySetpoints.TRENCH_LEFT_NEUTRAL), "Exit Aliance Left");
        else addAction(getAutoAlignmentCommand(isBlue, onTheFlySetpoints.TRENCH_RIGHT_NEUTRAL), "Exit Aliance Right");
    }
    
    public void addSweep(autoOptions startSide, autoOptions sweepAlignment){
        if(startSide == autoOptions.BORDER_LEFT){
            if(sweepAlignment == autoOptions.SWEEP_EDGE){
                addAction(SequentialPathGenerator.getSequentialPath(isBlue, onTheFlySetpoints.NEUTRAL_EDGE_LEFT, onTheFlySetpoints.NEUTRAL_EDGE_MID_FROM_LEFT)
                .raceWith(this.intake.setIntakeMaxLength().alongWith(this.intake.runIntake(RotationsPerSecond.of(60))).alongWith(this.indexer.runIndexer())), "Sweep Edge Left");
            }
            else{
                addAction(SequentialPathGenerator.getSequentialPath(isBlue, onTheFlySetpoints.NEUTRAL_CENTER_LEFT, onTheFlySetpoints.NEUTRAL_CENTER_MID_FROM_LEFT)
                .raceWith(this.intake.setIntakeMaxLength().alongWith(this.intake.runIntake(RotationsPerSecond.of(60))).alongWith(this.indexer.runIndexer())), "Sweep Center Left");
            }
        }
        else{
            if(sweepAlignment == autoOptions.SWEEP_EDGE){
                addAction(SequentialPathGenerator.getSequentialPath(isBlue, onTheFlySetpoints.NEUTRAL_EDGE_RIGHT, onTheFlySetpoints.NEUTRAL_EDGE_MID_FROM_RIGHT)
                .raceWith(this.intake.setIntakeMaxLength().alongWith(this.intake.runIntake(RotationsPerSecond.of(60))).alongWith(this.indexer.runIndexer())), "Sweep Edge Right");
            }
            else{
                addAction(SequentialPathGenerator.getSequentialPath(isBlue, onTheFlySetpoints.NEUTRAL_CENTER_RIGHT, onTheFlySetpoints.NEUTRAL_CENTER_MID_FROM_RIGHT)
                .raceWith(this.intake.setIntakeMaxLength().alongWith(this.intake.runIntake(RotationsPerSecond.of(60))).alongWith(this.indexer.runIndexer())), "Sweep Center Right");
            }
        }
    }

    public void addReturnAlliance(autoOptions returnSide, autoOptions returnLocation){
        if(returnSide == autoOptions.BORDER_LEFT){
            if(returnLocation == autoOptions.TRENCH){
                addAction(SequentialPathGenerator.getSequentialPath(isBlue, onTheFlySetpoints.TRENCH_LEFT_NEUTRAL, onTheFlySetpoints.TRENCH_LEFT_ALLIANCE), "Return Left Through Trench");
            
            }
            else{
                addAction(SequentialPathGenerator.getSequentialPath(isBlue, onTheFlySetpoints.RAMP_LEFT_NEUTRAL, onTheFlySetpoints.RAMP_LEFT_ALLIANCE), "Return Left Through Ramp");
            }
        }
        else{
            if(returnLocation == autoOptions.TRENCH){
                addAction(SequentialPathGenerator.getSequentialPath(isBlue, onTheFlySetpoints.TRENCH_RIGHT_NEUTRAL, onTheFlySetpoints.TRENCH_RIGHT_ALLIANCE), "Return Right Through Trench");
            
            }
            else{
                addAction(SequentialPathGenerator.getSequentialPath(isBlue, onTheFlySetpoints.RAMP_RIGHT_NEUTRAL, onTheFlySetpoints.RAMP_RIGHT_ALLIANCE), "Return Right Through Ramp");
            }
        }
        
    }

    public void addShootCommand(){
        /*addAction(
            intake.setIntakeMaxLength() // extend intake for maximum storage space
            // Run the spinner up to speed until it is at speed
            .alongWith(shooter.runSpinnerAdaptive(
                drive, drive.isBlue() 
                ? PointsOfInterest.centerOfHubBlue 
                : PointsOfInterest.centerOfHubRed))
            .alongWith(DriveCommands.autoJoystickDriveAtAngle(drive)) // Auto aim at the hub
            .until(() -> Math.abs(shooter.getSpinnerClosedLoopError()) < 1 && shooter.getLeftSpinnerVelocity().magnitude() > 30) // Run only the spin up and auto aim commands until the spinner is at speed
            .andThen(
                // Continue running spinner at speed
                shooter.runSpinnerAdaptive(
                    drive, drive.isBlue() 
                    ? PointsOfInterest.centerOfHubBlue 
                    : PointsOfInterest.centerOfHubRed)
                // Run indexer and kicker to feed shooter with fuel
                .alongWith(indexer.runIndexer()).withTimeout(6)
                .alongWith(shooter.runKicker()).withTimeout(6)
                // wait 4 seconds to fire majority of fuel, then retract intake to shove extra balls into the system
                .alongWith(
                    Commands.waitSeconds(4)
                    .andThen(intake.setIntakeMinLength())).withTimeout(6)), "Shoot");
        }*/
        addAction(
            intake.setIntakeMaxLength() // extend intake for maximum storage space
            // Run the spinner up to speed until it is at speed
            .alongWith(shooter.runSpinner()) // Auto aim at the hub
            .until(() -> Math.abs(shooter.getSpinnerClosedLoopError()) < 12 && shooter.getLeftSpinnerVelocity().magnitude() > 30) // Run only the spin up and auto aim commands until the spinner is at speed
            .andThen(
                // Continue running spinner at speed
                shooter.runSpinner() // Auto aim at the hub
                // Run indexer and kicker to feed shooter with fuel
                .alongWith(indexer.runIndexer()).withTimeout(6)
                .alongWith(shooter.runKicker()).withTimeout(6)
                // wait 4 seconds to fire majority of fuel, then retract intake to shove extra balls into the system
                .alongWith(
                    Commands.waitSeconds(4)
                    .andThen(intake.setIntakeMinLength())).withTimeout(6)), "Shoot");
        }

    public void addAlignScorePosition(autoOptions scoreLocation){
        switch (scoreLocation) {
            case SHOOT_LEFT:
                addAction(getAutoAlignmentCommand(isBlue, onTheFlySetpoints.SCORE_LEFT), "Align Shoot Left");
                break;

            case SHOOT_RIGHT:
                addAction(getAutoAlignmentCommand(isBlue, onTheFlySetpoints.SCORE_RIGHT), "Align Shoot Right");
                break;
            
            case SHOOT_CENTER:
                addAction(getAutoAlignmentCommand(isBlue, onTheFlySetpoints.SCORE_CENTER), "Align Shoot Center");
                break;
            default:
                break;
        }
    }
    
    public void addHumanPlayerCommand(autoOptions endScorePosition){
        addAction(getAutoAlignmentCommand(isBlue, onTheFlySetpoints.HUMAN_PLAYER), "Align To Human Player");
        addAction(Commands.waitSeconds(2), "Wait For HP");
        addAlignScorePosition(endScorePosition);
        addShootCommand();
    }

    public void addClimbCommand(autoOptions climbSide){
        // TODO add climb command
        if(climbSide == autoOptions.CLIMB_LEFT){
            addAction(SequentialPathGenerator.getSequentialPath(isBlue, onTheFlySetpoints.CLIMB_LEFT_SETUP, onTheFlySetpoints.CLIMB_LEFT_FINISH), "Align Climb Left");
            // add climb command
        }
        else {
            addAction(SequentialPathGenerator.getSequentialPath(isBlue, onTheFlySetpoints.CLIMB_RIGHT_SETUP, onTheFlySetpoints.CLIMB_RIGHT_FINISH), "Align Climb Right");
            // add climb command
        }
    }

    public void testAll(){
        this.addExitAlliance(autoOptions.BORDER_LEFT);
        //this.addExitAlliance(autoOptions.BORDER_RIGHT);

        //this.addSweep(autoOptions.BORDER_LEFT, autoOptions.SWEEP_CENTER);
        //this.addSweep(autoOptions.BORDER_LEFT, autoOptions.SWEEP_EDGE);
        //this.addSweep(autoOptions.BORDER_RIGHT, autoOptions.SWEEP_CENTER);
        this.addSweep(autoOptions.BORDER_RIGHT, autoOptions.SWEEP_EDGE);

        this.addShootCommand();

        //this.addReturnAlliance(autoOptions.BORDER_LEFT, autoOptions.TRENCH);
        //this.addReturnAlliance(autoOptions.BORDER_LEFT, autoOptions.RAMP);
        this.addReturnAlliance(autoOptions.BORDER_RIGHT, autoOptions.TRENCH);
        //this.addReturnAlliance(autoOptions.BORDER_RIGHT, autoOptions.RAMP);

        //this.addAlignScorePosition(autoOptions.SHOOT_LEFT);
        //this.addAlignScorePosition(autoOptions.SHOOT_CENTER);
        this.addAlignScorePosition(autoOptions.SHOOT_RIGHT);
        
        this.addShootCommand();

        //this.addClimbCommand(autoOptions.CLIMB_LEFT);
        //this.addClimbCommand(autoOptions.CLIMB_RIGHT);

        this.addHumanPlayerCommand(autoOptions.SHOOT_LEFT);
        this.addHumanPlayerCommand(autoOptions.SHOOT_CENTER);
        this.addHumanPlayerCommand(autoOptions.SHOOT_RIGHT);
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

    private Command getAutoAlignmentCommand(boolean isBlue, onTheFlySetpoints setpoint){
        Pose2d targetPose;
        if(isBlue) targetPose = setpoint.blueAlignmentPose;
        else targetPose = setpoint.redAlignmentPose;

        return AutoBuilder.pathfindToPose(
            targetPose,
            OperatorConstants.pathfindingConstraints);
    }

    private void addAction(Command command, String commandName){
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