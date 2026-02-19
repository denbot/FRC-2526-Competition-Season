package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Inches;
import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.OperatorConstants;

public class IntakeIOTalonFX implements IntakeIO {
    private final DigitalInput intakeDeployedSensor = new DigitalInput(IntakeConstants.DEPLOYED_DIO_PORT);
    private final DigitalInput intakeRetractedSensor = new DigitalInput(IntakeConstants.RETRACTED_DIO_PORT);
    
    private final TalonFX intakeMotor =
        new TalonFX(IntakeConstants.INTAKE_MOTOR_ID, OperatorConstants.canivoreCANBus);

    private final TalonFX rackMotor =
        new TalonFX(IntakeConstants.RACK_MOTOR_ID, OperatorConstants.canivoreCANBus);

    private final Debouncer intakeMotorDebounce = new Debouncer(0.5);
    private final Debouncer rackMotorDebounce = new Debouncer(0.5);

    private final StatusSignal<AngularVelocity> intakeVelocity = intakeMotor.getVelocity();
    private final StatusSignal<Current> intakeCurrentAmps = intakeMotor.getSupplyCurrent();
    private final StatusSignal<Current> intakeStallCurrentAmps = intakeMotor.getMotorStallCurrent();
    private final StatusSignal<Angle> intakePositionRot = intakeMotor.getPosition();

    private final StatusSignal<AngularVelocity> rackVelocity = rackMotor.getVelocity();
    private final StatusSignal<Current> rackCurrentAmps = rackMotor.getSupplyCurrent();
    private final StatusSignal<Current> rackStallCurrentAmps = rackMotor.getMotorStallCurrent();
    private final StatusSignal<Angle> rackPositionRot = rackMotor.getPosition();

    public IntakeIOTalonFX() {
        var intakeMotorConfig =
            new TalonFXConfiguration()
                .withFeedback(
                    new FeedbackConfigs()
                        .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor))
                .withSlot0(
                    new Slot0Configs()
                        .withKP(0.02) // TODO
                        .withKV(0.12) // TODO
                        .withKG(0)); // TODO

        var rackMotorConfig =
            new TalonFXConfiguration()
                .withFeedback(
                    new FeedbackConfigs()
                        .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor))
                .withSlot0(
                    new Slot0Configs()
                        .withKP(0) // TODO
                        .withKD(0) // TODO
                        .withKG(0)); // TODO

        intakeMotor.setNeutralMode(NeutralModeValue.Coast);
        rackMotor.setNeutralMode(NeutralModeValue.Brake);
        tryUntilOk(
            5,
            () ->
                intakeMotor
                    .getConfigurator()
                    .apply(intakeMotorConfig, 0.25)); // TODO: Unaware of what this does.
        tryUntilOk(
            5,
            () ->
                rackMotor
                    .getConfigurator()
                    .apply(rackMotorConfig, 0.25)); // TODO: Unaware of what this does.

        BaseStatusSignal.setUpdateFrequencyForAll(
            intakeMotor.getIsProLicensed().getValue() ? 200 : 50, intakeVelocity, intakeCurrentAmps, intakeStallCurrentAmps, intakePositionRot);

        BaseStatusSignal.setUpdateFrequencyForAll(
            rackMotor.getIsProLicensed().getValue() ? 200 : 50, rackVelocity, rackCurrentAmps, rackStallCurrentAmps, rackPositionRot);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        var intakeMotorStatus = BaseStatusSignal.refreshAll(intakeVelocity, intakeCurrentAmps, intakeStallCurrentAmps, intakePositionRot);
        var rackMotorStatus = BaseStatusSignal.refreshAll(rackVelocity, rackCurrentAmps, rackStallCurrentAmps, rackPositionRot);

        inputs.intakeMotorConnected = intakeMotorDebounce.calculate(intakeMotorStatus.isOK());
        inputs.rackMotorConnected = rackMotorDebounce.calculate(rackMotorStatus.isOK());

        inputs.intakeVelocityRotPerSec = intakeVelocity.getValue();
        inputs.rackVelocityRotPerSec = rackVelocity.getValue();

        inputs.intakePositionRots = intakePositionRot.getValue();
        inputs.rackPositionRots = rackPositionRot.getValue();

        inputs.stallCurrentIntake = intakeStallCurrentAmps.getValue();
        inputs.stallCurrentRack = rackStallCurrentAmps.getValue();

        inputs.intakeDeployedSwitch = !intakeDeployedSensor.get();
        inputs.intakeRetractedSwitch = !intakeRetractedSensor.get();
    }

    public void setIntakeVelocity(AngularVelocity velocity) {
        intakeMotor.setControl(new VelocityVoltage(velocity));
    }

    public void setIntakeExtensionLength(Distance length) {
        rackMotor.setControl(new PositionVoltage(IntakeConstants.intakeRotationsToRackRatio * length.abs(Inches)));
    }

    public void setIntakeMaxLength() {
        rackMotor.setControl(new PositionVoltage(IntakeConstants.intakeRotationsToRackRatio * IntakeConstants.intakeMaxExtensionLength));
    }

    public void setIntakeMinLength() {
        rackMotor.setControl(new PositionVoltage(IntakeConstants.intakeRotationsToRackRatio * IntakeConstants.intakeMinExtensionLength));
    }

    public void stopIntake() {
        intakeMotor.setControl(new CoastOut());
    }
}
