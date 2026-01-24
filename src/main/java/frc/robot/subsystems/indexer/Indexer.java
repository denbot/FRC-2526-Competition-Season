package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase{
    private final IndexerIO io;
    private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

    private AngularVelocity indexMotorSpeedSetpoint = RotationsPerSecond.zero();

    public Indexer(IndexerIO io){
        this.io = io;
    }

    @Override
    public void periodic(){
        // Log key variables
        io.updateInputs(inputs);
        Logger.processInputs("Indexer", inputs);
        Logger.recordOutput("Indexer Speed Setpoint", indexMotorSpeedSetpoint);
    }

    public Command runIndexer(){
        return Commands.runEnd(()-> this.io.runIndexerAtSpeed(indexMotorSpeedSetpoint), ()-> this.io.stopIntake());
    }

    public Command stopIndexer(){
        return Commands.runOnce(()-> this.io.stopIntake());
    }

    public void setIndexMotorSpeedSetpoint(AngularVelocity speed){
        indexMotorSpeedSetpoint = speed;
    }

    public void stepIndexMotorSpeedSetpoint(AngularVelocity speed){
        indexMotorSpeedSetpoint = indexMotorSpeedSetpoint.plus(speed);
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
