package frc.robot.subsystems.auto;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class AutoRoutineBuilder {

    private ArrayList<autoAction> actions = new ArrayList<>();
    private Intake intake;
    private Shooter shooter;
    private Indexer indexer;

    public AutoRoutineBuilder(Intake intake, Shooter shooter, Indexer indexer){
        this.intake = intake;
        this.shooter = shooter;
        this.indexer = indexer;
    }

    public enum actionType{
        ALIGN,
        SHOOTER,
        INTAKE,
        INDEXER;
    }

    private class autoAction{
        
        public actionType type;
        public Command command;

        public autoAction(actionType type, Command command){
            this.type = type;
            this.command = command;
        }
    }

    public Command getAutoRoutine(){
        SequentialCommandGroup finalRoutine = new SequentialCommandGroup();
        ParallelCommandGroup newCommand = new ParallelCommandGroup();
        for(autoAction action: actions){
            if(action.type == actionType.ALIGN){
                finalRoutine.addCommands(newCommand);
                newCommand = new ParallelCommandGroup(action.command);
            }            
            else{
                newCommand.addCommands(action.command);
            }
        }
        finalRoutine.addCommands(newCommand);
        return newCommand;
    }
}