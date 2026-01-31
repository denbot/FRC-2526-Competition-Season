package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOSim implements IntakeIO {
    private static final DCMotor intakeMotor = DCMotor.getKrakenX60Foc(1);
    private static final DCMotor rackMotor = DCMotor.getKrakenX60Foc(1);

    private DCMotorSim intakeMotorSim;
    private DCMotorSim rackMotorSim;

    private SimpleMotorFeedforward spinnFeedforwards = new SimpleMotorFeedforward(0, 0); // TODO
    private PIDController intakeController = new PIDController(1, 0, 0); // TODO
    private PIDController rackController = new PIDController(1, 0, 0); // TODO

    private double intakeAppliedVolts = 0.0;
    private double rackAppliedVolts = 0.0;

    public IntakeIOSim(){
        intakeMotorSim = 
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(intakeMotor, 0.2, 1), // TODO
            intakeMotor);   
        
        rackMotorSim = 
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(rackMotor, 0.2, 1), // TODO
            rackMotor);   
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs){
        intakeAppliedVolts = intakeController.calculate(intakeMotorSim.getAngularVelocityRPM()) + spinnFeedforwards.calculate(intakeMotorSim.getAngularVelocityRPM());
        rackAppliedVolts = rackController.calculate(rackMotorSim.getAngularVelocityRPM());

        intakeMotorSim.setInputVoltage(intakeAppliedVolts);
        rackMotorSim.setInputVoltage(rackAppliedVolts);

        intakeMotorSim.update(0.02);
        rackMotorSim.update(0.02);

        inputs.intakeMotorConnected = true;
        inputs.rackMotorConnected = true;

        inputs.intakeVelocityRotPerSec = intakeMotorSim.getAngularVelocity();
        inputs.rackVelocityRotPerSec = rackMotorSim.getAngularVelocity();

        inputs.intakePositionRots = Angle.ofBaseUnits(intakeMotorSim.getAngularPositionRotations(), Rotations);
        inputs.rackPositionRots =  Angle.ofBaseUnits(rackMotorSim.getAngularPositionRotations(), Rotations);

        inputs.stallCurrentIntake = null;
        inputs.stallCurrentRack = null;

        if (rackController.getSetpoint() == (IntakeConstants.intakeRotationsToRackRatio * IntakeConstants.intakeMaxExtensionLength)) {
            inputs.intakeDeployedSwitch = true;
        }

        if (rackController.getSetpoint() == (IntakeConstants.intakeRotationsToRackRatio * IntakeConstants.intakeMinExtensionLength)) {
            inputs.intakeRetractedSwitch = true;
        }
    }

    public void setIntakeVelocity(AngularVelocity velocity) {
        intakeController.setSetpoint(velocity.in(RotationsPerSecond));
    }

    public void setIntakeExtensionLength(Distance length) {
        rackController.setSetpoint(IntakeConstants.intakeRotationsToRackRatio * length.abs(Inches)); // TODO: This doesnt work
    }

    public void setIntakeMaxLength() {
        rackController.setSetpoint(IntakeConstants.intakeRotationsToRackRatio * IntakeConstants.intakeMaxExtensionLength); // TODO: This doesnt work
    }

    public void setIntakeMinLength() {
        rackController.setSetpoint(IntakeConstants.intakeRotationsToRackRatio * IntakeConstants.intakeMinExtensionLength); // TODO: This doesnt work
    }

    public void stopIntake() {
        intakeController.setSetpoint(0);
    }

    public void stopIntakeExtension() {
        rackController.setSetpoint(0);
    }
}
