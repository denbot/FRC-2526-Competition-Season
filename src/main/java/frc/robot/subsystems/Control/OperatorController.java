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
    //Sequence Buttons
    private final Trigger neutralZoneFeedButton = operatorController1.button(1);
    private final Trigger neutralZoneScoreTrenchButton = operatorController1.button(2);
    private final Trigger humanPlayerButton = operatorController1.button(3);
    private final Trigger neutralZoneScoreRampButton = operatorController1.button(4);
    private final Trigger aimAndShootButton = operatorController1.button(5);
    private final Trigger clearAllButton = operatorController1.button(6);
    private final Trigger leftRightSwitch = operatorController1.button(8);
    private final Trigger trenchBumpSwitch = operatorController1.button(12);
    // Control switches and buttons
    public final Trigger edgeSwitch = operatorController2.button(3);
    public final Trigger farSwitch = operatorController2.button(2);
    public final Trigger enableButtonBoxSwitch = operatorController2.button(12);
    public final Trigger churnTrigger = operatorController1.button(7);
    
    public OperatorController(AutoRoutineBuilder autoBuilder){

        // Add neutral sweep + score  
        neutralZoneScoreTrenchButton.onTrue(Commands.runOnce(
            () -> {
                System.out.println("Added neutral score to auto routine, exit trench");
                autoOptions startSide = leftRightSwitch.getAsBoolean() ? autoOptions.BORDER_RIGHT : autoOptions.BORDER_LEFT;
                autoBuilder.addExitAllianceTrench(startSide);
                autoBuilder.addSweep(startSide, edgeSwitch.getAsBoolean() ? autoOptions.SWEEP_EDGE : farSwitch.getAsBoolean() ? autoOptions.SWEEP_FAR : autoOptions.SWEEP_CENTER);
                autoBuilder.addReturnAlliance(startSide, trenchBumpSwitch.getAsBoolean() ? autoOptions.TRENCH : autoOptions.RAMP);
                autoBuilder.addShootCommand(); 
                Logger.recordOutput("Last Button Box Command", autoBuilder.commandNamesAsStringArray().length + (": TRENCH TO "+ (trenchBumpSwitch.getAsBoolean() ? "TRENCH" : "RAMP")+" SCORE - " + (edgeSwitch.getAsBoolean() ? "EDGE" : farSwitch.getAsBoolean() ? "FAR" : "CENTER") +(leftRightSwitch.getAsBoolean()?" RIGHT":" LEFT")));
            }).ignoringDisable(true));

        // Add neutral sweep + feed 
        neutralZoneFeedButton.onTrue(Commands.runOnce(
            () -> {
                System.out.println("Added neutral feed to auto routine");
                autoOptions startSide = leftRightSwitch.getAsBoolean() ? autoOptions.BORDER_RIGHT : autoOptions.BORDER_LEFT;
                autoBuilder.addExitAllianceRamp(startSide);
                autoBuilder.addSweep(startSide, edgeSwitch.getAsBoolean() ? autoOptions.SWEEP_EDGE : farSwitch.getAsBoolean() ? autoOptions.SWEEP_FAR : autoOptions.SWEEP_CENTER);
                autoBuilder.addAction(autoBuilder.getChurnCommand().withTimeout(1), "churn");
                autoBuilder.addShootCommand(); 
                Logger.recordOutput("Last Button Box Command", autoBuilder.commandNamesAsStringArray().length + (": FEED " + (edgeSwitch.getAsBoolean() ? "EDGE" : farSwitch.getAsBoolean() ? "FAR" : "CENTER") +(leftRightSwitch.getAsBoolean()?" RIGHT":" LEFT")));
            }).ignoringDisable(true));

        // add human player command
        humanPlayerButton.onTrue(Commands.runOnce(
            () -> {
                System.out.println("Added human player to auto routine");
                autoBuilder.addHumanPlayerCommand(autoOptions.SHOOT_CENTER);
                Logger.recordOutput("Last Button Box Command", autoBuilder.commandNamesAsStringArray().length + ": Human Player + Score");
            }).ignoringDisable(true));
        
        // add neutral sweep + score over ramp
        neutralZoneScoreRampButton.onTrue(Commands.runOnce(
            () -> {
                System.out.println("Added neutral score to auto routine, exit ramp");
                autoOptions startSide = leftRightSwitch.getAsBoolean() ? autoOptions.BORDER_RIGHT : autoOptions.BORDER_LEFT;
                autoBuilder.addExitAllianceRamp(startSide);
                autoBuilder.addSweep(startSide, edgeSwitch.getAsBoolean() ? autoOptions.SWEEP_EDGE : farSwitch.getAsBoolean() ? autoOptions.SWEEP_FAR : autoOptions.SWEEP_CENTER);
                autoBuilder.addReturnAlliance(startSide, trenchBumpSwitch.getAsBoolean() ? autoOptions.TRENCH : autoOptions.RAMP);
                autoBuilder.addShootCommand(); 
                Logger.recordOutput("Last Button Box Command", autoBuilder.commandNamesAsStringArray().length + (": RAMP TO "+ (trenchBumpSwitch.getAsBoolean() ? "TRENCH" : "RAMP") + " SCORE - " + (edgeSwitch.getAsBoolean() ? "EDGE" : farSwitch.getAsBoolean() ? "FAR" : "CENTER") +(leftRightSwitch.getAsBoolean()?" RIGHT":" LEFT")));
            }).ignoringDisable(true));  
        
        // add aim and shoot command
        aimAndShootButton.onTrue(Commands.runOnce(
            () -> {
                System.out.println("Added aim & shoot to auto routine");
                autoBuilder.addShootCommand();
                Logger.recordOutput("Last Button Box Command", autoBuilder.commandNamesAsStringArray().length + ": Aim And Shoot");
            }).ignoringDisable(true));
    
        // clear routine 
        clearAllButton.onTrue(Commands.runOnce(
            () -> {
                System.out.println("Cleared auto routine");
                autoBuilder.clearRoutine();
                Logger.recordOutput("Last Button Box Command", autoBuilder.commandNamesAsStringArray().length + ": Cleared Auto Routine");
            }).ignoringDisable(true));

        
        operatorController1.axisGreaterThan(1, 0.5)
            .onTrue(Commands.runOnce(() ->
                autoBuilder.shooter.stepSpinnerVelocitySetpoint(RotationsPerSecond.of(2))));
                
        operatorController1.axisLessThan(1, -0.5)
            .onTrue(Commands.runOnce(() ->
                autoBuilder.shooter.stepSpinnerVelocitySetpoint(RotationsPerSecond.of(-2))));
    }
}
