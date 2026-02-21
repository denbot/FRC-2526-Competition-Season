package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOSim implements IntakeIO {
    private static final DCMotor intakeMotor = DCMotor.getKrakenX60Foc(1);
    private static final DCMotor extensionLeftMotor = DCMotor.getKrakenX60Foc(1);
    private static final DCMotor extensionRightMotor = DCMotor.getKrakenX60Foc(1);

    private DCMotorSim intakeMotorSim;
    private DCMotorSim extensionLeftMotorSim;
    private DCMotorSim extensionRightMotorSim;

    private SimpleMotorFeedforward spinFeedforwards = new SimpleMotorFeedforward(0.0, 0.1); // TODO
    private PIDController intakeController = new PIDController(0.5, 1, 0); // TODO
    private PIDController extensionLeftController = new PIDController(7, 0, 0); // TODO
    private PIDController extensionRightController = new PIDController(7, 0, 0); // TODO

    private double intakeAppliedVoltsPID = 0.0;
    private double intakeAppliedVoltsFF = 0.0;
    private double extensionLeftAppliedVolts = 0.0;
    private double extensionRightAppliedVolts = 0.0;

    public IntakeIOSim(){
        intakeMotorSim = 
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(intakeMotor, 0.02, 2), // TODO
            intakeMotor);   
        
        extensionLeftMotorSim = 
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(extensionLeftMotor, 0.02, 23), // TODO
            extensionLeftMotor);
            
        extensionRightMotorSim = 
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(extensionRightMotor, 0.02, 23), // TODO
            extensionRightMotor);   
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs){
        intakeAppliedVoltsPID = intakeController.calculate(intakeMotorSim.getAngularVelocity().in(RotationsPerSecond)) + intakeAppliedVoltsFF;

        extensionLeftAppliedVolts = extensionLeftController.calculate(extensionLeftMotorSim.getAngularPositionRotations());
        extensionRightAppliedVolts = extensionRightController.calculate(extensionRightMotorSim.getAngularPositionRotations());

        intakeMotorSim.setInputVoltage(intakeAppliedVoltsPID);
        extensionLeftMotorSim.setInputVoltage(extensionLeftAppliedVolts);
        extensionRightMotorSim.setInputVoltage(extensionRightAppliedVolts);

        intakeMotorSim.update(0.02);
        extensionLeftMotorSim.update(0.02);
        extensionRightMotorSim.update(0.02);

        inputs.intakeMotorConnected = true;
        inputs.extensionMotorLeftConnected = true;
        inputs.extensionMotorRightConnected = true;

        inputs.intakeVelocityClosedLoopError = intakeController.getError();

        inputs.intakeVelocityRotPerSec = intakeMotorSim.getAngularVelocity();
        inputs.extensionVelocityLeft = extensionLeftMotorSim.getAngularVelocity();
        inputs.extensionVelocityRight = extensionRightMotorSim.getAngularVelocity();

        inputs.intakePositionRots = Angle.ofBaseUnits(intakeMotorSim.getAngularPositionRotations(), Rotations);
        inputs.extensionLeftPositionRots =  Angle.ofBaseUnits(extensionLeftMotorSim.getAngularPositionRotations(), Rotations);
        inputs.extensionRightPositionRots =  Angle.ofBaseUnits(extensionRightMotorSim.getAngularPositionRotations(), Rotations);

        inputs.stallCurrentIntake = null;
        inputs.stallCurrentExtensionLeft = null;
        inputs.stallCurrentExtensionRight = null;

        if (extensionLeftController.getSetpoint() == (IntakeConstants.intakeMaxExtensionPosition.magnitude())
            || extensionLeftController.getSetpoint() == (IntakeConstants.intakeMaxExtensionPosition.magnitude())) {
            inputs.intakeDeployedSwitch = true;
        }

        if (extensionLeftController.getSetpoint() == (IntakeConstants.intakeMinExtensionPosition.magnitude())
            || extensionLeftController.getSetpoint() == (IntakeConstants.intakeMinExtensionPosition.magnitude())) {
            inputs.intakeRetractedSwitch = true;
        }
    }

    public void setIntakeVelocity(AngularVelocity velocity) {
        intakeAppliedVoltsFF = spinFeedforwards.calculate(velocity.in(RotationsPerSecond));
        intakeController.setSetpoint(velocity.in(RotationsPerSecond));
    }

    public void setIntakeExtension(Angle position) {
        extensionLeftController.setSetpoint(position.magnitude()); // TODO: This doesnt work
        extensionRightController.setSetpoint(position.magnitude()); // TODO: This doesnt work
    }

    public void setIntakeMaxLength() {
        extensionLeftController.setSetpoint(IntakeConstants.intakeMaxExtensionPosition.magnitude()); // TODO: This doesnt work
        extensionRightController.setSetpoint(IntakeConstants.intakeMaxExtensionPosition.magnitude()); // TODO: This doesnt work
    }

    public void setIntakeIdleLength() {
        extensionLeftController.setSetpoint(IntakeConstants.intakeIdleExtensionPosition.magnitude()); // TODO: This doesnt work
        extensionRightController.setSetpoint(IntakeConstants.intakeIdleExtensionPosition.magnitude()); // TODO: This doesnt work
    }

    public void setIntakeMinLength() {
        extensionLeftController.setSetpoint(IntakeConstants.intakeMinExtensionPosition.magnitude()); // TODO: This doesnt work
        extensionRightController.setSetpoint(IntakeConstants.intakeMinExtensionPosition.magnitude()); // TODO: This doesnt work
    }

    public void stopIntake() {
        intakeController.setSetpoint(0);
    }
}
