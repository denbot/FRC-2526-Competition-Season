package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ClimberIOSim implements ClimberIO{
    private static final DCMotor climberMotor = DCMotor.getKrakenX60(1);
    private DCMotorSim climberMotorSim;
    private SimpleMotorFeedforward climberFeedforward = new SimpleMotorFeedforward(0.0, 0.0);
    private PIDController climberMotorController = new PIDController(0.0, 0.0, 0.0);
    private double climberMotorAppliedVoltsPID = 0;
    private double climberMotorAppliedVoltsFF = 0;

    public ClimberIOSim(){
        climberMotorSim = 
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(climberMotor, 0.002, 2), 
            climberMotor);   
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs){
        climberMotorAppliedVoltsPID = climberMotorController.calculate(climberMotorSim.getAngularPosition().in(Degrees)) + climberMotorAppliedVoltsFF;
        climberMotorSim.setInputVoltage(climberMotorAppliedVoltsPID);
        climberMotorSim.update(0.02);
        inputs.climberMotorConnected = true;
        inputs.climberMotorAmps = Amp.of(climberMotorSim.getCurrentDrawAmps());
        inputs.climberMotorPosition = climberMotorSim.getAngularPosition();
        inputs.climberMotorClosedLoopError = climberMotorController.getError();
    }

    public void setClimberToSetpoint(Angle position){
        climberMotorAppliedVoltsFF = climberFeedforward.calculate(climberMotorSim.getAngularPosition().in(Degrees));
        climberMotorController.setSetpoint(position.in(Degrees));
    }

    public void setClimberMaxExtension(){
        climberMotorAppliedVoltsFF = climberFeedforward.calculate(climberMotorSim.getAngularPosition().in(Degrees));
        climberMotorController.setSetpoint(ClimberConstants.CLIMBER_MAX_EXTENSION_SETPOINT.magnitude());
    }
    public void setClimberMinExtension(Angle position){
        climberMotorAppliedVoltsFF = climberFeedforward.calculate(climberMotorSim.getAngularPosition().in(Degrees));
        climberMotorController.setSetpoint(ClimberConstants.CLIMBER_MIN_EXTENSION_SETPOINT.magnitude());
    }

}
