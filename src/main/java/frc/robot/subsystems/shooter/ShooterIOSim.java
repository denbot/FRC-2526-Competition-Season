package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ShooterIOSim implements ShooterIO{
    private static final DCMotor leftSpinnerMotor = DCMotor.getKrakenX60Foc(1);
    private static final DCMotor rightSpinnerMotor = DCMotor.getKrakenX60Foc(1);
    private static final DCMotor kickerMotor = DCMotor.getKrakenX60Foc(1);

    private DCMotorSim leftSpinnerMotorSim;
    private DCMotorSim rightSpinnerMotorSim;
    private DCMotorSim kickerMotorSim;

    private PIDController leftSpinnerController = new PIDController(0, 0, 0);
    private PIDController rightSpinnerController = new PIDController(0, 0, 0);
    private PIDController kickerController = new PIDController(0, 0, 0);

    private double leftSpinnerAppliedVolts = 0.0;
    private double rightSpinnerAppliedVolts = 0.0;
    private double kickerAppliedVolts = 0.0;

    public ShooterIOSim(){
        leftSpinnerMotorSim = 
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(leftSpinnerMotor, 0.1, 0.1), 
            leftSpinnerMotor);   
        
        rightSpinnerMotorSim = 
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(rightSpinnerMotor, 0.1, 0.1), 
            rightSpinnerMotor);   
        
        kickerMotorSim = 
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(kickerMotor, 0.1, 0.1), 
            kickerMotor);   
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs){
        leftSpinnerAppliedVolts = leftSpinnerController.calculate(leftSpinnerMotorSim.getAngularVelocity().in(RotationsPerSecond));
        rightSpinnerAppliedVolts = rightSpinnerController.calculate(rightSpinnerMotorSim.getAngularVelocity().in(RotationsPerSecond));
        kickerAppliedVolts = kickerController.calculate(kickerMotorSim.getAngularVelocity().in(RotationsPerSecond));

        leftSpinnerMotorSim.setInputVoltage(leftSpinnerAppliedVolts);
        rightSpinnerMotorSim.setInputVoltage(rightSpinnerAppliedVolts);
        kickerMotorSim.setInputVoltage(kickerAppliedVolts);

        leftSpinnerMotorSim.update(0.02);
        rightSpinnerMotorSim.update(0.02);
        kickerMotorSim.update(0.02);

        inputs.leftSpinnerMotorConnected = true;
        inputs.rightSpinnerMotorConnected = true;
        inputs.kickerMotorConnected = true;
        
        inputs.leftSpinnerRotationSpeed = RotationsPerSecond.of(leftSpinnerMotorSim.getAngularVelocityRPM() / 60);
        inputs.rightSpinnerRotationSpeed = RotationsPerSecond.of(rightSpinnerMotorSim.getAngularVelocityRPM() / 60);
        inputs.kickerRotationSpeed = RotationsPerSecond.of(kickerMotorSim.getAngularVelocityRPM() / 60);
        
        inputs.leftSpinnerCurrentAmps = Amp.of(leftSpinnerMotorSim.getCurrentDrawAmps());
        inputs.rightSpinnerCurrentAmps = Amp.of(rightSpinnerMotorSim.getCurrentDrawAmps());
        inputs.kickerCurrentAmps = Amp.of(kickerMotorSim.getCurrentDrawAmps());

        inputs.leftSpinnerPositionRots = leftSpinnerMotorSim.getAngularPosition();
        inputs.rightSpinnerPositionRots = rightSpinnerMotorSim.getAngularPosition();
        inputs.kickerPositionRots = kickerMotorSim.getAngularPosition();
        
        inputs.leftSpinnerClosedLoopError = leftSpinnerController.getError();
        inputs.rightSpinnerClosedLoopError = rightSpinnerController.getError();
        inputs.kickerClosedLoopError = kickerController.getError();
    }

    public void setLeftSpinnerVelocity(AngularVelocity velocity){
        leftSpinnerController.setSetpoint(velocity.in(RotationsPerSecond));
    }
    public void setRightSpinnerVelocity(AngularVelocity velocity){
        rightSpinnerController.setSetpoint(velocity.in(RotationsPerSecond));
    }
    public void stopLeftSpinner(){
        leftSpinnerController.setSetpoint(0);
    }
    public void stopRightSpinner(){
        rightSpinnerController.setSetpoint(0);
    }
    public void setKickerVelocity(AngularVelocity velocity){
        kickerController.setSetpoint(velocity.in(RotationsPerSecond));
    }
    public void stopKicker(){
        kickerController.setSetpoint(0);
    }
}
