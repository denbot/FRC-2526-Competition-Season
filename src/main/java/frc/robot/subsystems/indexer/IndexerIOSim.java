package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Kelvin;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IndexerIOSim implements IndexerIO{
    private static final DCMotor indexMotor = DCMotor.getKrakenX44(1);
    private DCMotorSim indexMotorSim;
    private SimpleMotorFeedforward indexFeedforward = new SimpleMotorFeedforward(0.0, 0.11500000059604645);
    private PIDController indexMotorController = new PIDController(0.001, 0.0, 0.0);
    private double indexMotorAppliedVoltsPID = 0;
    private double indexMotorAppliedVoltsFF = 0;

    public IndexerIOSim(){
        indexMotorSim = 
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(indexMotor, 0.002, 2), 
            indexMotor);   
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs){
        indexMotorAppliedVoltsPID = indexMotorController.calculate(indexMotorSim.getAngularVelocity().in(RotationsPerSecond)) + indexMotorAppliedVoltsFF;
        indexMotorSim.setInputVoltage(indexMotorAppliedVoltsPID);
        indexMotorSim.update(0.02);
        inputs.indexMotorConnected = true;
        inputs.indexMotorRotationSpeed = indexMotorSim.getAngularVelocity();
        inputs.indexMotorCurrentAmps = Amp.of(indexMotorSim.getCurrentDrawAmps());
        inputs.indexMotorPositionRots = indexMotorSim.getAngularPosition();
        inputs.indexMotorClosedLoopError = indexMotorController.getError();
        inputs.indexMotorTemperature = Kelvin.zero();
    }

    public void runIndexerAtSpeed(AngularVelocity speed){
        indexMotorAppliedVoltsFF = indexFeedforward.calculate(indexMotorSim.getAngularVelocity().in(RotationsPerSecond));
        indexMotorController.setSetpoint(speed.in(RotationsPerSecond));
    }
    
    public void stopIndexer(){
        indexMotorAppliedVoltsFF = indexFeedforward.calculate(0);
        indexMotorController.setSetpoint(0);
    }
}
