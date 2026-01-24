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

    private SimpleMotorFeedforward spinnFeedforwards = new SimpleMotorFeedforward(0.001, 0.11500000059604645);
    private PIDController spinnerController = new PIDController(0, 0, 0);
    private PIDController kickerController = new PIDController(0, 0, 0);

    private double spinnerAppliedVolts = 0.0;
    private double kickerAppliedVolts = 0.0;

    public ShooterIOSim(){
        spinnerMotorSim = 
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(spinnerMotor, 0.0002, 1), 
            spinnerMotor);   
        
        kickerMotorSim = 
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(kickerMotor, 0.0002, 1), 
            kickerMotor);   
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs){
        spinnerAppliedVolts = spinnerController.calculate(spinnerMotorSim.getAngularVelocityRPM()) + spinnFeedforwards.calculate(spinnerMotorSim.getAngularVelocityRPM());
        kickerAppliedVolts = kickerController.calculate(kickerMotorSim.getAngularVelocityRPM());

        spinnerMotorSim.setInputVoltage(spinnerAppliedVolts);
        kickerMotorSim.setInputVoltage(kickerAppliedVolts);

        spinnerMotorSim.update(0.02);
        kickerMotorSim.update(0.02);

        inputs.leftSpinnerMotorConnected = true;
        inputs.rightSpinnerMotorConnected = true;
        inputs.kickerMotorConnected = true;
        
        inputs.leftSpinnerRotationSpeed = RotationsPerSecond.of(spinnerMotorSim.getAngularVelocityRPM() / 60);
        inputs.kickerRotationSpeed = RotationsPerSecond.of(kickerMotorSim.getAngularVelocityRPM() / 60);
        
        inputs.leftSpinnerCurrentAmps = Amp.of(spinnerMotorSim.getCurrentDrawAmps());
        inputs.kickerCurrentAmps = Amp.of(kickerMotorSim.getCurrentDrawAmps());

        inputs.leftSpinnerPositionRots = spinnerMotorSim.getAngularPosition();
        inputs.kickerPositionRots = kickerMotorSim.getAngularPosition();
        
        inputs.leftSpinnerClosedLoopError = spinnerController.getError();
        inputs.kickerClosedLoopError = kickerController.getError();
    }

    public void setSpinnerVelocity(AngularVelocity velocity){
        spinnerController.setSetpoint(velocity.in(RotationsPerSecond));
    }
    public void stopSpinner(){
        spinnerController.setSetpoint(0);
    }

    public void setKickerVelocity(AngularVelocity velocity){
        kickerController.setSetpoint(velocity.in(RotationsPerSecond));
    }
    public void stopKicker(){
        kickerController.setSetpoint(0);
    }
}
