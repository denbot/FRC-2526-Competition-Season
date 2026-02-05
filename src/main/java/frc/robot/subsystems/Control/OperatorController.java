package frc.robot.subsystems.Control;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.robot.subsystems.auto.AutoRoutineBuilder;
import frc.robot.subsystems.auto.AutoRoutineBuilder.autoOptions;

public class OperatorController {
    private static final CommandGenericHID operatorController1 = new CommandGenericHID(1);
    
    public static void defineAutoBindings(AutoRoutineBuilder autoBuilder){
        // TODO all button numbers are assigned and not true to reality
        
        // add exit aliance based on state of side exit switch
        operatorController1.button(2).onTrue(Commands.runOnce(
            () ->{ 
                autoBuilder.addExitAlliance(
                    operatorController1.button(1).getAsBoolean() ? autoOptions.BORDER_LEFT : autoOptions.BORDER_RIGHT);
            }));
    
        // add center sweep based on state of exit switch and sweep switch
        operatorController1.button(4).onTrue(Commands.runOnce(
            () -> autoBuilder.addSweep(
                operatorController1.button(1).getAsBoolean() ? autoOptions.BORDER_LEFT : autoOptions.BORDER_RIGHT, 
                operatorController1.button(3).getAsBoolean() ? autoOptions.SWEEP_EDGE : autoOptions.SWEEP_CENTER)));
        
        // add center sweep based on state of exit switch and return switch
        operatorController1.button(6).onTrue(Commands.runOnce(
            () -> autoBuilder.addReturnAlliance(
                operatorController1.button(1).getAsBoolean() ? autoOptions.BORDER_LEFT : autoOptions.BORDER_RIGHT, 
                operatorController1.button(5).getAsBoolean() ? autoOptions.TRENCH : autoOptions.RAMP)));

        // add exit, sweep, return, shoot based on all switch values
        operatorController1.button(7).onTrue(Commands.runOnce(
            () -> {
                autoOptions startSide = operatorController1.button(1).getAsBoolean() ? autoOptions.BORDER_LEFT : autoOptions.BORDER_RIGHT;
                autoBuilder.addExitAlliance(startSide);
                autoBuilder.addSweep(startSide, operatorController1.button(3).getAsBoolean() ? autoOptions.SWEEP_EDGE : autoOptions.SWEEP_CENTER); 
                autoBuilder.addReturnAlliance(startSide, operatorController1.button(5).getAsBoolean() ? autoOptions.TRENCH : autoOptions.RAMP);
                autoBuilder.addScoreCommand();
        })); 

        // add feed command
        operatorController1.button(8).onTrue(Commands.runOnce(
            () -> autoBuilder.addFeedCommand()));        

        // add human player command
        operatorController1.button(9).onTrue(Commands.runOnce(
            () -> autoBuilder.addHumanPlayerCommand(autoOptions.SHOOT_RIGHT)));
        
        // add climb command
        operatorController1.button(11).onTrue(Commands.runOnce(
            () -> autoBuilder.addHumanPlayerCommand(
                operatorController1.button(10).getAsBoolean() ? autoOptions.CLIMB_LEFT : autoOptions.CLIMB_RIGHT)));  
                
        // clear routine 
        operatorController1.button(12).onTrue(Commands.runOnce(
            () -> autoBuilder.clearRoutine()));

        // remove last command from routine 
        /* Commented out because im annoyed we cant fit all commands on one operator controller
        operatorController1.button(12).onTrue(Commands.runOnce(
            () -> autoBuilder.removeLast()));
             */
        
    }
}
