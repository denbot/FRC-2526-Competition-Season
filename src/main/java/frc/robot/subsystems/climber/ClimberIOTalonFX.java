package frc.robot.subsystems.climber;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import frc.robot.Constants.OperatorConstants;

public class ClimberIOTalonFX implements ClimberIO{
    private final TalonFX climberMotor = 
        new TalonFX(ClimberConstants.CLIMBER_MOTOR_ID, OperatorConstants.canivoreCANBus);
     private final Debouncer climberMotorDebouncer = new Debouncer(0.5);

    private final StatusSignal<AngularVelocity> climberMotorVelocity = climberMotor.getVelocity();
    private final StatusSignal<Current> climberMotorCurrentAmps = climberMotor.getSupplyCurrent();
    private final StatusSignal<Angle> climberMotorPositionRots = climberMotor.getPosition();
    private final StatusSignal<Double> climberMotorClosedLoopError = climberMotor.getClosedLoopError();
    
    public ClimberIOTalonFX() {
        var climberMotorConfig =
        new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimitEnable(true)
                    .withStatorCurrentLimit(40))
            .withFeedback(
                new FeedbackConfigs()
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor))
            .withSlot0(
                new Slot0Configs()
                    .withKP(.001)
                    .withKV(0.1));

        climberMotor.setNeutralMode(NeutralModeValue.Coast);
        tryUntilOk(5, () -> climberMotor.getConfigurator().apply(climberMotorConfig, 0.25));

        
        BaseStatusSignal.setUpdateFrequencyForAll(
            climberMotor.getIsProLicensed().getValue() ? 200 : 50, 
            climberMotorVelocity, 
            climberMotorCurrentAmps);
    }
    @Override
    public void updateInputs(ClimberIOInputs inputs){
        var climberMotorMotorStatus = BaseStatusSignal.refreshAll(climberMotorVelocity, climberMotorCurrentAmps);

        inputs.climberMotorConnected = climberMotorDebouncer.calculate(climberMotorMotorStatus.isOK());
        inputs.climberMotorPosition = climberMotorPositionRots.getValue();
        inputs.climberMotorClosedLoopError = climberMotorClosedLoopError.getValueAsDouble();
        inputs.climberMotorAmps = climberMotorCurrentAmps.getValue();
    }

    public void setClimberToSetpoint(Angle speed){
        climberMotor.setControl(new PositionVoltage(speed));
    }

    public void setClimberMaxExtension(){
        climberMotor.setControl(new PositionVoltage(ClimberConstants.CLIMBER_MAX_EXTENSION_LENGTH.magnitude() * ClimberConstants.CLIMBER_ROTATION_TO_METER_RATIO));
    }

    public void setClimberMinExtension(){
        climberMotor.setControl(new PositionVoltage(ClimberConstants.CLIMBER_MIN_EXTENSION_LENGTH.magnitude() * ClimberConstants.CLIMBER_ROTATION_TO_METER_RATIO));
    }
}
