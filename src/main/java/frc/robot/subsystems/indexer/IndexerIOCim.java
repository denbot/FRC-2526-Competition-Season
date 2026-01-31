package frc.robot.subsystems.indexer;


import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.units.measure.AngularVelocity;

public class IndexerIOCim implements IndexerIO{
    private final TalonSRX indexMotor = 
        new TalonSRX(IndexerConstants.INDEX_MOTOR_ID);

    public IndexerIOCim() {}

    @Override
    public void updateInputs(IndexerIOInputs inputs){}

    public void runIndexerAtSpeed(AngularVelocity percent){
        indexMotor.set(TalonSRXControlMode.PercentOutput, percent.in(RotationsPerSecond)/100);
    }

    public void stopIndexer(){
        indexMotor.set(TalonSRXControlMode.PercentOutput, 0);
    }
}
