package frc.robot.subsystems.control;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.auto.AutoRoutineCreator;
import frc.robot.subsystems.auto.AutoRoutineCreator.driverRelative;
import frc.robot.subsystems.auto.AutoCommandHelper;

public class OperatorController {
    private final CommandGenericHID operatorController1 = new CommandGenericHID(1);
    private final CommandGenericHID operatorController2 = new CommandGenericHID(2);
    private final Trigger leftRightSwitch = operatorController1.button(8);
    private final Trigger edgeCenterSwitch = operatorController1.button(11);
    private final Trigger trenchBumpSwitch = operatorController1.button(12);
    private final Trigger neutralZoneFeedButton = operatorController1.button(1);
    private final Trigger neutralZoneScoreButton = operatorController1.button(2);
    private final Trigger humanPlayerButton = operatorController1.button(3);
    private final Trigger climbButton = operatorController1.button(4);
    private final Trigger aimAndShootButton = operatorController1.button(5);
    private final Trigger clearAllButton = operatorController1.button(6);
    private final Trigger clearLastButton = operatorController1.button(7);
    private final Trigger teleopAutoClimbSwitch = operatorController2.button(12);
    // 3-way rotary switch, toggles A when left, neither when center, B when right
    public final Trigger blueWonAutoToggle = operatorController2.button(2);
    public final Trigger redWonAutoToggle = operatorController2.button(3);
    private static boolean isBlue = true;
    
    public OperatorController(){

        AutoCommandHelper.saveState(); // ensure there is a basic empty state to load

        blueWonAutoToggle.whileTrue(Commands.runOnce(() -> {AutoRoutineCreator.setIsBlue(true); isBlue = true; SmartDashboard.putString("Current Team", isBlue?"blue":"red");}).ignoringDisable(true));
        redWonAutoToggle.whileTrue(Commands.runOnce(() -> {AutoRoutineCreator.setIsBlue(false); isBlue = false;SmartDashboard.putString("Current Team", isBlue?"blue":"red");}).ignoringDisable(true));
        // Add neutral sweep + score  

        trenchBumpSwitch.onChange(Commands.runOnce(() -> AutoRoutineCreator.setStartingSide(leftRightSwitch.getAsBoolean() ? driverRelative.RIGHT : driverRelative.LEFT)));

        neutralZoneScoreButton.onTrue(Commands.runOnce(
            () -> {
                System.out.println("Added neutral score to auto routine");
                AutoRoutineCreator.addExitAlliance(trenchBumpSwitch.getAsBoolean() ? driverRelative.TRENCH : driverRelative.RAMP);
                AutoRoutineCreator.addSweep(edgeCenterSwitch.getAsBoolean() ? driverRelative.FAR : driverRelative.CLOSE, true);
                AutoRoutineCreator.addReturnAlliance(trenchBumpSwitch.getAsBoolean() ? driverRelative.TRENCH : driverRelative.RAMP);
                AutoRoutineCreator.addAlignScorePosition(leftRightSwitch.getAsBoolean() ? driverRelative.RIGHT : driverRelative.LEFT);
                AutoRoutineCreator.addShootCommand(); 
                AutoCommandHelper.saveState();
            }).ignoringDisable(true));

        // Add neutral sweep + feed 
        neutralZoneFeedButton.onTrue(Commands.runOnce(
            () -> {
                System.out.println("Added neutral feed to auto routine");
                driverRelative startSide = leftRightSwitch.getAsBoolean() ? driverRelative.RIGHT : driverRelative.LEFT;
                AutoRoutineCreator.addExitAlliance(startSide);
                AutoRoutineCreator.addSweep(edgeCenterSwitch.getAsBoolean() ? driverRelative.FAR : driverRelative.CLOSE, false);
                AutoRoutineCreator.addShootCommand(); 
                AutoCommandHelper.saveState();
            }).ignoringDisable(true));

        // add human player command
        humanPlayerButton.onTrue(Commands.runOnce(
            () -> {
                System.out.println("Added human player to auto routine");
                AutoRoutineCreator.addHumanPlayerCommand();
                AutoCommandHelper.saveState();
            }).ignoringDisable(true));
        
        // add climb command
        climbButton.onTrue(Commands.runOnce(
            () -> {
                System.out.println("Added climb to auto routine");
                AutoRoutineCreator.addClimbCommand(
                    leftRightSwitch.getAsBoolean() ? driverRelative.RIGHT : driverRelative.LEFT);
                AutoCommandHelper.saveState();
            }).ignoringDisable(true));  
        
        // add aim and shoot command
        aimAndShootButton.onTrue(Commands.runOnce(
            () -> {
                System.out.println("Added aim & shoot to auto routine");
                AutoRoutineCreator.addShootCommand();
                AutoCommandHelper.saveState();
            }).ignoringDisable(true));
    
        // clear routine 
        clearAllButton.onTrue(Commands.runOnce(
            () -> {
                System.out.println("Cleared auto routine");
                AutoRoutineCreator.clearAll();
                AutoCommandHelper.saveState();
            }).ignoringDisable(true));

        // remove last command from routine 
        clearLastButton.onTrue(Commands.runOnce(
            () -> {
                System.out.println("Cleared auto routine");
                AutoRoutineCreator.removeLast();
            }).ignoringDisable(true));
        
        operatorController1.axisGreaterThan(1, 0.5)
            .onTrue(Commands.runOnce(() ->
                AutoRoutineCreator.shooter.stepSpinnerVelocitySetpoint(RotationsPerSecond.of(2))));
                
        operatorController1.axisLessThan(1, -0.5)
            .onTrue(Commands.runOnce(() ->
                AutoRoutineCreator.shooter.stepSpinnerVelocitySetpoint(RotationsPerSecond.of(-2))));
    }

    public static boolean getIsBlue(){
        return isBlue;
    }
}
