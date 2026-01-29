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
    private SimpleMotorFeedforward indexFeedforward = new SimpleMotorFeedforward(0.001, 0.11500000059604645);
    private PIDController indexMotorController = new PIDController(0.11, 0.0, 0.0);
    private double indexMotorAppliedVolts = 0;

    public IndexerIOSim(){
        indexMotorSim = 
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(indexMotor, 0.0002, 1), 
            indexMotor);   
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs){
        indexMotorAppliedVolts = indexMotorController.calculate(indexMotorSim.getAngularVelocityRPM()) + indexFeedforward.calculate(indexMotorSim.getAngularVelocityRPM());
        indexMotorSim.setInputVoltage(indexMotorAppliedVolts);
        indexMotorSim.update(0.02);
        inputs.indexMotorConnected = true;
        inputs.indexMotorRotationSpeed = RotationsPerSecond.of(indexMotorSim.getAngularVelocityRPM() / 60);
        inputs.indexMotorCurrentAmps = Amp.of(indexMotorSim.getCurrentDrawAmps());
        inputs.indexMotorPositionRots = indexMotorSim.getAngularPosition();
        inputs.indexMotorClosedLoopError = indexMotorController.getError();
        inputs.indexMotorTemperature = Kelvin.zero();
    }

    public void runIndexerAtSpeed(AngularVelocity speed){
        indexMotorController.setSetpoint(speed.in(RotationsPerSecond));
    }
    
    public void stopIndexer(){
        indexMotorController.setSetpoint(0);
    }
}
