package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ShooterIOSim implements ShooterIO{
    private static final DCMotor spinnerMotor = DCMotor.getKrakenX60Foc(2);
    private static final DCMotor kickerMotor = DCMotor.getKrakenX60Foc(1);

    private DCMotorSim spinnerMotorSim;
    private DCMotorSim kickerMotorSim;

    private SimpleMotorFeedforward spinFeedforwards = new SimpleMotorFeedforward(0.0703125, 0.1200000059604645);
    private SimpleMotorFeedforward kickerFeedforwards = new SimpleMotorFeedforward(0.09, 0.25);
    private PIDController spinnerController = new PIDController(0.1, 0, 0);
    private PIDController kickerController = new PIDController(0.01, 0, 0.0);

    private double spinnerAppliedVoltsPID = 0.0;
    private double spinnerAppliedVoltsFF = 0.0;
    private double kickerAppliedVoltsPID = 0.0;
    private double kickerAppliedVoltsFF = 0.0;

    public ShooterIOSim(){
        spinnerMotorSim = 
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(spinnerMotor, 0.02, 1), 
            spinnerMotor);   
        
        kickerMotorSim = 
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(kickerMotor, 0.002, 1), 
            kickerMotor);   
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs){
        spinnerAppliedVoltsPID = spinnerController.calculate(spinnerMotorSim.getAngularVelocity().in(RotationsPerSecond)) + spinnerAppliedVoltsFF;
        kickerAppliedVoltsPID = kickerController.calculate(kickerMotorSim.getAngularVelocity().in(RotationsPerSecond)) + kickerAppliedVoltsFF;

        spinnerMotorSim.setInputVoltage(spinnerAppliedVoltsPID);
        kickerMotorSim.setInputVoltage(kickerAppliedVoltsPID);

        spinnerMotorSim.update(0.02);
        kickerMotorSim.update(0.02);

        inputs.leftSpinnerMotorConnected = true;
        inputs.rightSpinnerMotorConnected = true;
        inputs.kickerMotorConnected = true;
        
        inputs.leftSpinnerRotationSpeed = spinnerMotorSim.getAngularVelocity();
        inputs.kickerRotationSpeed = kickerMotorSim.getAngularVelocity();
        
        inputs.leftSpinnerCurrentAmps = Amp.of(spinnerMotorSim.getCurrentDrawAmps());
        inputs.kickerCurrentAmps = Amp.of(kickerMotorSim.getCurrentDrawAmps());

        inputs.leftSpinnerPositionRots = spinnerMotorSim.getAngularPosition();
        inputs.kickerPositionRots = kickerMotorSim.getAngularPosition();
        
        inputs.leftSpinnerClosedLoopError = spinnerController.getError();
        inputs.kickerClosedLoopError = kickerController.getError();
    }

    public void setSpinnerVelocity(AngularVelocity velocity){
        spinnerAppliedVoltsFF = spinFeedforwards.calculate(velocity.in(RotationsPerSecond));
        spinnerController.setSetpoint(velocity.in(RotationsPerSecond));
    }
    public void stopSpinner(){
        spinnerAppliedVoltsFF = spinFeedforwards.calculate(0);
        spinnerController.setSetpoint(0);
    }

    public void setKickerVelocity(AngularVelocity velocity){
        kickerAppliedVoltsFF = kickerFeedforwards.calculate(velocity.in(RotationsPerSecond));
        kickerController.setSetpoint(velocity.in(RotationsPerSecond));
    }
    public void stopKicker(){
        kickerAppliedVoltsFF = kickerFeedforwards.calculate(0);
        kickerController.setSetpoint(0);
    }
}
