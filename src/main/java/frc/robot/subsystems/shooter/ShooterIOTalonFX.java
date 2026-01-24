package frc.robot.subsystems.shooter;

import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

//import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
//import frc.robot.subsystems.shooter.ShooterConstants.OperatorConstants;
import frc.robot.subsystems.shooter.ShooterConstants.OperatorConstants;

public class ShooterIOTalonFX implements ShooterIO {
    private final TalonFX leftSpinnerMotor =
        new TalonFX(ShooterConstants.LEFT_SPINNER_MOTOR_ID, OperatorConstants.canivoreSerial);

    private final TalonFX rightSpinnerMotor =
        new TalonFX(ShooterConstants.RIGHT_SPINNER_MOTOR_ID, OperatorConstants.canivoreSerial);
    
    private final TalonFX kickerMotor =
        new TalonFX(ShooterConstants.KICKER_MOTOR_ID);

    private final Debouncer leftSpinnerMotorDebounce = new Debouncer(0.5);
    private final Debouncer rightSpinnerMotorDebounce = new Debouncer(0.5);
    private final Debouncer kickerMotorDebounce = new Debouncer(0.5);

    /* Unused FOC Tuning
    private static final VelocityTorqueCurrentFOC leftSpinnerFOC =
        new VelocityTorqueCurrentFOC(0).withAcceleration(ShooterConstants.spinnerAcceleration);
    private static final VelocityTorqueCurrentFOC rightSpinnerFOC =
        new VelocityTorqueCurrentFOC(0).withAcceleration(ShooterConstants.spinnerAcceleration);
    private static final VelocityTorqueCurrentFOC kickerFOC =
        new VelocityTorqueCurrentFOC(0).withAcceleration(ShooterConstants.kickerAcceleration);
    */

    private final StatusSignal<AngularVelocity> leftSpinnerVelocity = leftSpinnerMotor.getVelocity();
    private final StatusSignal<Current> leftSpinnerCurrentAmps = leftSpinnerMotor.getSupplyCurrent();

    private final StatusSignal<AngularVelocity> rightSpinnerVelocity = rightSpinnerMotor.getVelocity();
    private final StatusSignal<Current> rightSpinnerCurrentAmps = rightSpinnerMotor.getSupplyCurrent();

    private final StatusSignal<AngularVelocity> kickerVelocity = kickerMotor.getVelocity();
    private final StatusSignal<Current> kickerCurrentAmps = kickerMotor.getSupplyCurrent();

    private final StatusSignal<Angle> leftSpinnerPositionRot = leftSpinnerMotor.getPosition();
    private final StatusSignal<Double> leftSpinnerClosedLoopError = leftSpinnerMotor.getClosedLoopError();

    private final StatusSignal<Angle> rightSpinnerPositionRot = rightSpinnerMotor.getPosition();
    private final StatusSignal<Double> rightSpinnerClosedLoopError = rightSpinnerMotor.getClosedLoopError();

    private final StatusSignal<Angle> kickerPositionRot = kickerMotor.getPosition();
    private final StatusSignal<Double> kickerClosedLoopError = kickerMotor.getClosedLoopError();

    public ShooterIOTalonFX() {
        var leftSpinnerMotorConfig =
        new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimitEnable(true)
                    .withStatorCurrentLimit(70))
            .withFeedback(
                new FeedbackConfigs()
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor))
            .withSlot0(
                new Slot0Configs()
                    .withKP(.001)
                    .withKS(0.0703125)
                    .withKV(0.11500000059604645));

        var rightSpinnerMotorConfig =
        new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimitEnable(true)
                    .withStatorCurrentLimit(70))
            .withFeedback(
                new FeedbackConfigs()
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor))
            .withSlot0(
                new Slot0Configs()
                    .withKP(.001)
                    .withKS(0.0703125)
                    .withKV(0.11500000059604645));

        var kickerMotorConfig =
        new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimitEnable(true)
                    .withStatorCurrentLimit(70))
            .withFeedback(
                new FeedbackConfigs()
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor))
            .withSlot0(
                new Slot0Configs()
                    .withKP(45)
                    .withKD(0)
                    .withKG(0.2));


        leftSpinnerMotor.setNeutralMode(NeutralModeValue.Coast);
        rightSpinnerMotor.setNeutralMode(NeutralModeValue.Coast);
        kickerMotor.setNeutralMode(NeutralModeValue.Coast);
        tryUntilOk(5, () -> leftSpinnerMotor.getConfigurator().apply(leftSpinnerMotorConfig, 0.25));
        tryUntilOk(5, () -> rightSpinnerMotor.getConfigurator().apply(rightSpinnerMotorConfig, 0.25));
        tryUntilOk(5, () -> kickerMotor.getConfigurator().apply(kickerMotorConfig, 0.25));

        BaseStatusSignal.setUpdateFrequencyForAll(
            leftSpinnerMotor.getIsProLicensed().getValue() ? 200 : 50, 
            leftSpinnerVelocity, 
            leftSpinnerCurrentAmps);
        
        BaseStatusSignal.setUpdateFrequencyForAll(
            rightSpinnerMotor.getIsProLicensed().getValue() ? 200 : 50, 
            rightSpinnerVelocity, 
            rightSpinnerCurrentAmps);
            
        BaseStatusSignal.setUpdateFrequencyForAll(
            kickerMotor.getIsProLicensed().getValue() ? 200 : 50, 
            kickerVelocity, 
            kickerCurrentAmps);
    
        rightSpinnerMotor.setControl(new Follower(leftSpinnerMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs){
        var leftSpinnerMotorStatus = BaseStatusSignal.refreshAll(leftSpinnerVelocity, leftSpinnerCurrentAmps);
        var rightSpinnerMotorStatus = BaseStatusSignal.refreshAll(rightSpinnerVelocity, rightSpinnerCurrentAmps);
        var kickerMotorStatus = BaseStatusSignal.refreshAll(kickerVelocity, kickerCurrentAmps);

        inputs.leftSpinnerMotorConnected = leftSpinnerMotorDebounce.calculate(leftSpinnerMotorStatus.isOK());
        inputs.rightSpinnerMotorConnected = rightSpinnerMotorDebounce.calculate(rightSpinnerMotorStatus.isOK());
        inputs.kickerMotorConnected = kickerMotorDebounce.calculate(kickerMotorStatus.isOK());

        inputs.leftSpinnerRotationSpeed = leftSpinnerVelocity.getValue();
        inputs.rightSpinnerRotationSpeed = rightSpinnerVelocity.getValue();
        inputs.kickerRotationSpeed = kickerVelocity.getValue();

        inputs.leftSpinnerPositionRots = leftSpinnerPositionRot.getValue();
        inputs.rightSpinnerPositionRots = rightSpinnerPositionRot.getValue();
        inputs.kickerPositionRots = kickerPositionRot.getValue();

        inputs.leftSpinnerClosedLoopError = leftSpinnerClosedLoopError.getValueAsDouble();
        inputs.rightSpinnerClosedLoopError = rightSpinnerClosedLoopError.getValueAsDouble();
        inputs.kickerClosedLoopError = kickerClosedLoopError.getValueAsDouble();

        inputs.leftSpinnerCurrentAmps = leftSpinnerCurrentAmps.getValue();
        inputs.rightSpinnerCurrentAmps = rightSpinnerCurrentAmps.getValue();
        inputs.kickerCurrentAmps = kickerCurrentAmps.getValue();
    }

    public void setSpinnerVelocity(AngularVelocity velocity){
        leftSpinnerMotor.setControl(new VelocityVoltage(velocity));
    }

    public void stopSpinner(){
        leftSpinnerMotor.setControl(new CoastOut());
    }

    public void setKickerVelocity(AngularVelocity velocity){
        kickerMotor.setControl(new VelocityVoltage(velocity));
    }
    public void stopKicker(){
        kickerMotor.setControl(new CoastOut());
    }
}