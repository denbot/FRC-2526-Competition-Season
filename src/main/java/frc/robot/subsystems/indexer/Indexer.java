package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.state.IndexerState;
import frc.robot.state.RebuiltStateMachine;

public class Indexer extends SubsystemBase{
    private final IndexerIO io;
    private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

    private AngularVelocity indexMotorSpeedSetpoint = RotationsPerSecond.of(30);

    public Indexer(IndexerIO io, RebuiltStateMachine stateMachine){
        this.io = io;

        stateMachine
            .state(IndexerState.STOPPED)
            .to(IndexerState.RUNNING)
            .run(runIndexer());
        stateMachine
            .state(IndexerState.REVERSING)
            .to(IndexerState.RUNNING)
            .run(runIndexer());
        
        stateMachine
            .state(IndexerState.STOPPED)
            .to(IndexerState.REVERSING)
            .run(reverseIndexer());
        stateMachine
            .state(IndexerState.RUNNING)
            .to(IndexerState.REVERSING)
            .run(reverseIndexer());
        
        stateMachine
            .state(IndexerState.RUNNING)
            .to(IndexerState.STOPPED)
            .run(stopIndexer());
        stateMachine
            .state(IndexerState.REVERSING)
            .to(IndexerState.STOPPED)
            .run(stopIndexer());
    }

    @Override
    public void periodic(){
        // Log key variables
        io.updateInputs(inputs);
        Logger.processInputs("Indexer", inputs);
        Logger.recordOutput("Indexer Speed Setpoint", indexMotorSpeedSetpoint);
    }

    public Command runIndexer(){
        return Commands.repeatingSequence(Commands.runOnce(()-> this.io.runIndexerAtSpeed(indexMotorSpeedSetpoint)), Commands.waitSeconds(0.875), Commands.runOnce(()-> this.io.stopIndexer()), Commands.waitSeconds(0.125)).finallyDo(() -> this.io.stopIndexer());
    }

    public Command reverseIndexer(){
        return Commands.runEnd(()-> this.io.runIndexerAtSpeed(indexMotorSpeedSetpoint.times(-1)), ()-> this.io.stopIndexer());
    }

    public Command stopIndexer(){
        return Commands.runOnce(()-> this.io.stopIndexer());
    }

    public void setIndexMotorSpeedSetpoint(AngularVelocity speed){
        indexMotorSpeedSetpoint = speed;
    }

    public Boolean getIndexMotorConnected(){
        return inputs.indexMotorConnected;
    }

    public AngularVelocity getIndexMotorRotationSpeed(){
        return inputs.indexMotorRotationSpeed;
    }

    public Angle getIndexMotorPosition(){
        return inputs.indexMotorPositionRots;
    }

    public double getIndexMotorClosedLoopError(){
        return inputs.indexMotorClosedLoopError;
    }

    public Current getIndexMotorCurrent(){
        return inputs.indexMotorCurrentAmps;
    }
}
