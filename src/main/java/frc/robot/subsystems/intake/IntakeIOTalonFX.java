package frc.robot.subsystems.intake;

import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.OperatorConstants;

public class IntakeIOTalonFX implements IntakeIO {
    private final DigitalInput intakeDeployedSensor = new DigitalInput(IntakeConstants.DEPLOYED_DIO_PORT);
    private final DigitalInput intakeRetractedSensor = new DigitalInput(IntakeConstants.RETRACTED_DIO_PORT);
    
    private final TalonFX intakeMotor =
        new TalonFX(IntakeConstants.INTAKE_MOTOR_ID, OperatorConstants.canivoreCANBus);

    private final TalonFX extensionMotorLeft =
        new TalonFX(IntakeConstants.EXTENSION_MOTOR_LEFT_ID, OperatorConstants.canivoreCANBus);
   
        private final TalonFX extensionMotorRight =
        new TalonFX(IntakeConstants.EXTENSION_MOTOR_RIGHT_ID, OperatorConstants.canivoreCANBus);

    private final Debouncer intakeMotorDebounce = new Debouncer(0.5);
    private final Debouncer extensionMotorLeftDebounce = new Debouncer(0.5);
    private final Debouncer extensionMotorRightDebounce = new Debouncer(0.5);

    private final StatusSignal<AngularVelocity> intakeVelocity = intakeMotor.getVelocity();
    private final StatusSignal<Current> intakeCurrentAmps = intakeMotor.getSupplyCurrent();
    private final StatusSignal<Current> intakeStallCurrentAmps = intakeMotor.getMotorStallCurrent();
    private final StatusSignal<Angle> intakePositionRot = intakeMotor.getPosition();

    private final StatusSignal<AngularVelocity> extensionLeftVelocity = extensionMotorLeft.getVelocity();
    private final StatusSignal<AngularVelocity> extensionRightVelocity = extensionMotorRight.getVelocity();
    private final StatusSignal<Current> extensionLeftCurrentAmps = extensionMotorLeft.getSupplyCurrent();
    private final StatusSignal<Current> extensionRightCurrentAmps = extensionMotorRight.getSupplyCurrent();
    private final StatusSignal<Current> extensionLeftStallCurrentAmps = extensionMotorLeft.getMotorStallCurrent();
    private final StatusSignal<Current> extensionRightStallCurrentAmps = extensionMotorRight.getMotorStallCurrent();
    private final StatusSignal<Angle> extensionLeftPositionRot = extensionMotorLeft.getPosition();
    private final StatusSignal<Angle> extensionRightPositionRot = extensionMotorRight.getPosition();

    public IntakeIOTalonFX() {
        var intakeMotorConfig =
            new TalonFXConfiguration()
                .withFeedback(
                    new FeedbackConfigs()
                        .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor))
                .withSlot0(
                    new Slot0Configs()
                        .withKP(.5)
                        .withKI(1)
                        .withKV(0.1));

        var extensionMotorLeftConfig =
            new TalonFXConfiguration()
                .withFeedback(
                    new FeedbackConfigs()
                        .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor))
                .withSlot0(
                    new Slot0Configs()
                        .withKP(0) // TODO
                        .withKD(0) // TODO
                        .withKG(0)); // TODO

        var extensionMotorRightConfig =
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

        extensionMotorLeft.setNeutralMode(NeutralModeValue.Brake);
        extensionMotorRight.setNeutralMode(NeutralModeValue.Brake);

        tryUntilOk(
            5,
            () ->
                intakeMotor
                    .getConfigurator()
                    .apply(intakeMotorConfig, 0.25)); // TODO: Unaware of what this does.
        tryUntilOk(
            5,
            () ->
                extensionMotorLeft
                    .getConfigurator()
                    .apply(extensionMotorLeftConfig, 0.25)); // TODO: Unaware of what this does.
        tryUntilOk(
            5,
            () ->
                extensionMotorRight
                    .getConfigurator()
                    .apply(extensionMotorRightConfig, 0.25)); // TODO: Unaware of what this does.


        BaseStatusSignal.setUpdateFrequencyForAll(
            intakeMotor.getIsProLicensed().getValue() ? 200 : 50, intakeVelocity, intakeCurrentAmps, intakeStallCurrentAmps, intakePositionRot);

        BaseStatusSignal.setUpdateFrequencyForAll(
            extensionMotorLeft.getIsProLicensed().getValue() ? 200 : 50, extensionLeftVelocity, extensionLeftCurrentAmps, extensionLeftStallCurrentAmps, extensionLeftPositionRot);
        
        BaseStatusSignal.setUpdateFrequencyForAll(
            extensionMotorRight.getIsProLicensed().getValue() ? 200 : 50, extensionRightVelocity, extensionRightCurrentAmps, extensionRightStallCurrentAmps, extensionRightPositionRot);

        extensionMotorRight.setControl(new Follower(extensionMotorLeft.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        var intakeMotorStatus = BaseStatusSignal.refreshAll(intakeVelocity, intakeCurrentAmps, intakeStallCurrentAmps, intakePositionRot);
        var intakeLeftMotorStatus = BaseStatusSignal.refreshAll(extensionLeftVelocity, extensionLeftCurrentAmps, extensionLeftStallCurrentAmps, extensionLeftPositionRot);
        var intakeRightMotorStatus = BaseStatusSignal.refreshAll(extensionRightVelocity, extensionRightCurrentAmps, extensionRightStallCurrentAmps, extensionRightPositionRot);

        inputs.intakeMotorConnected = intakeMotorDebounce.calculate(intakeMotorStatus.isOK());
        inputs.extensionMotorLeftConnected = extensionMotorLeftDebounce.calculate(intakeLeftMotorStatus.isOK());
        inputs.extensionMotorRightConnected = extensionMotorRightDebounce.calculate(intakeRightMotorStatus.isOK());

        inputs.intakeVelocityRotPerSec = intakeVelocity.getValue();
        inputs.extensionVelocityLeft = extensionLeftVelocity.getValue();
        inputs.extensionVelocityRight = extensionRightVelocity.getValue();

        inputs.intakePositionRots = intakePositionRot.getValue();
        inputs.extensionLeftPositionRots = extensionLeftPositionRot.getValue();
        inputs.extensionRightPositionRots = extensionRightPositionRot.getValue();

        inputs.stallCurrentIntake = intakeStallCurrentAmps.getValue();
        inputs.stallCurrentExtensionLeft = extensionLeftStallCurrentAmps.getValue();
        inputs.stallCurrentExtensionRight = extensionRightStallCurrentAmps.getValue();

        inputs.intakeDeployedSwitch = !intakeDeployedSensor.get();
        inputs.intakeRetractedSwitch = !intakeRetractedSensor.get();
    }

    public void setIntakeVelocity(AngularVelocity velocity) {
        intakeMotor.setControl(new VelocityVoltage(velocity));
    }

    public void setIntakeExtensionLength(Angle position) {
        extensionMotorLeft.setControl(new PositionVoltage(position));
    }

    public void setIntakeMaxLength() {
        extensionMotorLeft.setControl(new PositionVoltage(IntakeConstants.intakeMaxExtensionPosition));
    }
    
    public void setIntakeIdleLength() {
        extensionMotorLeft.setControl(new PositionVoltage(IntakeConstants.intakeIdleExtensionPosition));
    }

    public void setIntakeMinLength() {
        extensionMotorLeft.setControl(new PositionVoltage(IntakeConstants.intakeMinExtensionPosition));
    }

    public void stopIntake() {
        intakeMotor.setControl(new CoastOut());
    }
}
