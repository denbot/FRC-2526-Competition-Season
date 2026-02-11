package frc.robot.subsystems.indexer;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import frc.robot.Constants.OperatorConstants;

public class IndexerIOTalonFX implements IndexerIO{
    private final TalonFX indexMotor = 
        new TalonFX(IndexerConstants.INDEX_MOTOR_ID, OperatorConstants.canivoreCANBus);
    
    private final Debouncer indexMotorDebouncer = new Debouncer(0.5);

    private final StatusSignal<AngularVelocity> indexMotorVelocity = indexMotor.getVelocity();
    private final StatusSignal<Current> indexMotorCurrentAmps = indexMotor.getSupplyCurrent();
    private final StatusSignal<Angle> indexMotorPositionRots = indexMotor.getPosition();
    private final StatusSignal<Double> indexMotorClosedLoopError = indexMotor.getClosedLoopError();
    private final StatusSignal<Temperature> indexMotorTemperature = indexMotor.getDeviceTemp();
    
    public IndexerIOTalonFX() {
        var indexMotorConfig =
        new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive))
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
                    .withKS(0.0)
                    .withKV(0.1));

        indexMotor.setNeutralMode(NeutralModeValue.Coast);
        tryUntilOk(5, () -> indexMotor.getConfigurator().apply(indexMotorConfig, 0.25));

        
        BaseStatusSignal.setUpdateFrequencyForAll(
            indexMotor.getIsProLicensed().getValue() ? 200 : 50, 
            indexMotorVelocity, 
            indexMotorCurrentAmps);
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs){
        var indexMotorMotorStatus = BaseStatusSignal.refreshAll(indexMotorVelocity, indexMotorCurrentAmps);

        inputs.indexMotorConnected = indexMotorDebouncer.calculate(indexMotorMotorStatus.isOK());
        inputs.indexMotorRotationSpeed = indexMotorVelocity.getValue();
        inputs.indexMotorPositionRots = indexMotorPositionRots.getValue();
        inputs.indexMotorClosedLoopError = indexMotorClosedLoopError.getValueAsDouble();
        inputs.indexMotorCurrentAmps = indexMotorCurrentAmps.getValue();
        inputs.indexMotorTemperature = indexMotorTemperature.getValue();
    }

    public void runIndexerAtSpeed(AngularVelocity speed){
        indexMotor.setControl(new VelocityVoltage(speed));
    }

    public void stopIndexer(){
        indexMotor.setControl(new CoastOut());
    }
}
