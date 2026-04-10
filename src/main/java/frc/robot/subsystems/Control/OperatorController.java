package frc.robot.subsystems.control;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.auto.AutoRoutineBuilder;
import frc.robot.subsystems.auto.AutoRoutineBuilder.autoOptions;

public class OperatorController {
    private final CommandGenericHID operatorController1 = new CommandGenericHID(1);
    private final CommandGenericHID operatorController2 = new CommandGenericHID(2);
    private final Trigger leftRightSwitch = operatorController1.button(8);
    private final Trigger edgeCenterSwitch = operatorController1.button(11);
    private final Trigger trenchBumpSwitch = operatorController1.button(12);
    private final Trigger neutralZoneFeedButton = operatorController1.button(1);
    private final Trigger neutralZoneScoreTrenchButton = operatorController1.button(2);
    private final Trigger humanPlayerButton = operatorController1.button(3);
    private final Trigger neutralZoneScoreRampButton = operatorController1.button(4);
    private final Trigger aimAndShootButton = operatorController1.button(5);
    private final Trigger clearAllButton = operatorController1.button(6);
    private final Trigger clearLastButton = operatorController1.button(7);
    public final Trigger churnTrigger = operatorController2.button(12);
    // 3-way rotary switch, toggles A when left, neither when center, B when right
    public final Trigger blueWonAutoToggle = operatorController2.button(2);
    public final Trigger redWonAutoToggle = operatorController2.button(3);
    private static boolean isBlue = true;
    
    public OperatorController(AutoRoutineBuilder autoBuilder){

        blueWonAutoToggle.whileTrue(Commands.runOnce(() -> {autoBuilder.setIsBlue(true); isBlue = true; SmartDashboard.putString("Current Team", isBlue?"blue":"red");Logger.recordOutput("Last Button Box Command", "Set Side to Blue");}).ignoringDisable(true));
        redWonAutoToggle.whileTrue(Commands.runOnce(() -> {autoBuilder.setIsBlue(false); isBlue = false;SmartDashboard.putString("Current Team", isBlue?"blue":"red");Logger.recordOutput("Last Button Box Command", "Set Side to Red");}).ignoringDisable(true));
        // Add neutral sweep + score  
        neutralZoneScoreTrenchButton.onTrue(Commands.runOnce(
            () -> {
                System.out.println("Added neutral score to auto routine, exit trench");
                autoOptions startSide = leftRightSwitch.getAsBoolean() ? autoOptions.BORDER_RIGHT : autoOptions.BORDER_LEFT;
                autoBuilder.addExitAllianceTrench(startSide);
                autoBuilder.addSweep(startSide, edgeCenterSwitch.getAsBoolean() ? autoOptions.SWEEP_CENTER : autoOptions.SWEEP_EDGE);
                autoBuilder.addReturnAlliance(startSide, trenchBumpSwitch.getAsBoolean() ? autoOptions.TRENCH : autoOptions.RAMP);
                // autoBuilder.addAlignScorePosition(leftRightSwitch.getAsBoolean() ? autoOptions.SHOOT_RIGHT : autoOptions.SHOOT_LEFT);
                autoBuilder.addShootCommand(); 
                Logger.recordOutput("Last Button Box Command", (leftRightSwitch.getAsBoolean()?"Right":"Left")+" Side Trench Exit Neutral Sweep Return Through "+(trenchBumpSwitch.getAsBoolean()?"Trench":"Ramp")+" Then Score");
            }).ignoringDisable(true));

        // Add neutral sweep + feed 
        neutralZoneFeedButton.onTrue(Commands.runOnce(
            () -> {
                System.out.println("Added neutral feed to auto routine");
                autoOptions startSide = leftRightSwitch.getAsBoolean() ? autoOptions.BORDER_RIGHT : autoOptions.BORDER_LEFT;
                autoBuilder.addExitAllianceTrench(startSide);
                autoBuilder.addSweep(startSide, edgeCenterSwitch.getAsBoolean() ? autoOptions.SWEEP_CENTER : autoOptions.SWEEP_EDGE);
                autoBuilder.addShootCommand(); 
                Logger.recordOutput("Last Button Box Command", ((leftRightSwitch.getAsBoolean()?"Right":"Left"))+" Side Trench Exit Neutral Sweep+Feed");
            }).ignoringDisable(true));

        // add human player command
        humanPlayerButton.onTrue(Commands.runOnce(
            () -> {
                System.out.println("Added human player to auto routine");
                autoBuilder.addHumanPlayerCommand(autoOptions.SHOOT_CENTER);
                Logger.recordOutput("Last Button Box Command", "Human Player + Score");
            }).ignoringDisable(true));
        
        // add climb command
        neutralZoneScoreRampButton.onTrue(Commands.runOnce(
            () -> {
                System.out.println("Added neutral score to auto routine, exit ramp");
                autoOptions startSide = leftRightSwitch.getAsBoolean() ? autoOptions.BORDER_RIGHT : autoOptions.BORDER_LEFT;
                autoBuilder.addExitAllianceRamp(startSide);
                autoBuilder.addSweep(startSide, edgeCenterSwitch.getAsBoolean() ? autoOptions.SWEEP_CENTER : autoOptions.SWEEP_EDGE);
                autoBuilder.addReturnAlliance(startSide, trenchBumpSwitch.getAsBoolean() ? autoOptions.TRENCH : autoOptions.RAMP);
                // autoBuilder.addAlignScorePosition(leftRightSwitch.getAsBoolean() ? autoOptions.SHOOT_RIGHT : autoOptions.SHOOT_LEFT);
                autoBuilder.addShootCommand(); 
                Logger.recordOutput("Last Button Box Command", (leftRightSwitch.getAsBoolean()?"Right":"Left")+" Side Ramp Exit Neutral Sweep Return Through "+(trenchBumpSwitch.getAsBoolean()?"Trench":"Ramp")+" Then Score");
            }).ignoringDisable(true));  
        
        // add aim and shoot command
        aimAndShootButton.onTrue(Commands.runOnce(
            () -> {
                System.out.println("Added aim & shoot to auto routine");
                autoBuilder.addShootCommand();
                Logger.recordOutput("Last Button Box Command", "Aim And Shoot");
            }).ignoringDisable(true));
    
        // clear routine 
        clearAllButton.onTrue(Commands.runOnce(
            () -> {
                System.out.println("Cleared auto routine");
                autoBuilder.clearRoutine();
                Logger.recordOutput("Last Button Box Command", "Cleared Auto Routine");
            }).ignoringDisable(true));

        // remove last command from routine 
        clearLastButton.onTrue(Commands.runOnce(
            () -> {
                System.out.println("Cleared auto routine");
                autoBuilder.removeLast();
                Logger.recordOutput("Last Button Box Command", "Removed Last Command From Routine");
            }).ignoringDisable(true));
        
        operatorController1.axisGreaterThan(1, 0.5)
            .onTrue(Commands.runOnce(() ->
                autoBuilder.shooter.stepSpinnerVelocitySetpoint(RotationsPerSecond.of(2))));
                
        operatorController1.axisLessThan(1, -0.5)
            .onTrue(Commands.runOnce(() ->
                autoBuilder.shooter.stepSpinnerVelocitySetpoint(RotationsPerSecond.of(-2))));
    }
    public static boolean getIsBlue(){
        return isBlue;
    }
}
