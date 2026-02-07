package frc.robot.subsystems.Control;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.state.HubState;
import frc.robot.subsystems.auto.AutoRoutineBuilder;
import frc.robot.subsystems.auto.AutoRoutineBuilder.autoOptions;

public class OperatorController {
    private final CommandGenericHID operatorController1 = new CommandGenericHID(1);
    private final Trigger leftRightSwitch = operatorController1.button(12);
    private final Trigger edgeCenterSwitch = operatorController1.button(11);
    private final Trigger trenchBumpSwitch = operatorController1.button(8);
    private final Trigger neutralZoneFeedButton = operatorController1.button(1);
    private final Trigger neutralZoneScoreButton = operatorController1.button(2);
    private final Trigger humanPlayerButton = operatorController1.button(3);
    private final Trigger climbButton = operatorController1.button(4);
    private final Trigger aimAndShootButton = operatorController1.button(5);
    private final Trigger clearAllButton = operatorController1.button(6);
    private final Trigger clearLastButton = operatorController1.button(7);
    private final Trigger setWeWonAutoPointsTrueButton = operatorController1.button(9);
    private final Trigger setWeWonAutoPointsFalseButton = operatorController1.button(10);
    
    public OperatorController(AutoRoutineBuilder autoBuilder){
        // Add neutral sweep + score  
        neutralZoneScoreButton.onTrue(Commands.runOnce(
            () -> {
                System.out.println("Added neutral score to auto routine");
                autoOptions startSide = leftRightSwitch.getAsBoolean() ? autoOptions.BORDER_RIGHT : autoOptions.BORDER_LEFT;
                autoBuilder.addExitAlliance(startSide);
                autoBuilder.addSweep(startSide, edgeCenterSwitch.getAsBoolean() ? autoOptions.SWEEP_CENTER : autoOptions.SWEEP_EDGE);
                autoBuilder.addReturnAlliance(startSide, trenchBumpSwitch.getAsBoolean() ? autoOptions.RAMP : autoOptions.TRENCH);
                autoBuilder.addScoreCommand(); 
            }).ignoringDisable(true));

        // Add neutral sweep + feed 
        neutralZoneFeedButton.onTrue(Commands.runOnce(
            () -> {
                System.out.println("Added neutral feed to auto routine");
                autoOptions startSide = leftRightSwitch.getAsBoolean() ? autoOptions.BORDER_RIGHT : autoOptions.BORDER_LEFT;
                autoBuilder.addExitAlliance(startSide);
                autoBuilder.addSweep(startSide, edgeCenterSwitch.getAsBoolean() ? autoOptions.SWEEP_CENTER : autoOptions.SWEEP_EDGE);
                autoBuilder.addFeedCommand(); 
            }).ignoringDisable(true));

        // add human player command
        humanPlayerButton.onTrue(Commands.runOnce(
            () -> {
                System.out.println("Added human player to auto routine");
                autoBuilder.addHumanPlayerCommand(autoOptions.SHOOT_RIGHT);
            }).ignoringDisable(true));
        
        // add climb command
        climbButton.onTrue(Commands.runOnce(
            () -> {
                System.out.println("Added climb to auto routine");
                autoBuilder.addClimbCommand(
                    leftRightSwitch.getAsBoolean() ? autoOptions.CLIMB_RIGHT : autoOptions.CLIMB_LEFT);
            }).ignoringDisable(true));  
        
        // add aim and shoot command
        aimAndShootButton.onTrue(Commands.runOnce(
            () -> {
                System.out.println("Added aim & shoot to auto routine");
                autoBuilder.addScoreCommand();
            }).ignoringDisable(true));
    
        // clear routine 
        clearAllButton.onTrue(Commands.runOnce(
            () -> {
                System.out.println("Cleared auto routine");
                autoBuilder.clearRoutine();
            }).ignoringDisable(true));

        // remove last command from routine 
        clearLastButton.onTrue(Commands.runOnce(
            () -> {
                System.out.println("Cleared auto routine");
                autoBuilder.removeLast();
            }).ignoringDisable(true));

        setWeWonAutoPointsTrueButton.onTrue(Commands.runOnce(
            () -> {
                System.out.println("We won auto points");
                HubState.configureShifts(true);
            }));

        setWeWonAutoPointsFalseButton.onTrue(Commands.runOnce(
            () -> {
                System.out.println("We lost auto points");
                HubState.configureShifts(false);
            }));
    }
}
