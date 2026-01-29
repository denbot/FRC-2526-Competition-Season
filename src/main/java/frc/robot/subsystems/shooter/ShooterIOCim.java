package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.units.measure.AngularVelocity;

public class ShooterIOCim implements ShooterIO {
    private final TalonSRX leftSpinnerMotor =
        new TalonSRX(ShooterConstants.RIGHT_SPINNER_MOTOR_ID);

    private final TalonSRX rightSpinnerMotor =
        new TalonSRX(ShooterConstants.RIGHT_SPINNER_MOTOR_ID);
    
    private final TalonSRX kickerMotor =
        new TalonSRX(ShooterConstants.KICKER_MOTOR_ID);

    public ShooterIOCim() {}

    @Override
    public void updateInputs(ShooterIOInputs inputs){}

    public void setSpinnerVelocity(AngularVelocity velocity){
        leftSpinnerMotor.set(TalonSRXControlMode.PercentOutput, velocity.in(RotationsPerSecond) / 100);
        rightSpinnerMotor.set(TalonSRXControlMode.PercentOutput, -velocity.in(RotationsPerSecond) / 100);
    }

    public void stopSpinner(){
        rightSpinnerMotor.set(TalonSRXControlMode.PercentOutput, 0);
        rightSpinnerMotor.set(TalonSRXControlMode.PercentOutput, 0);
    }

    public void setKickerVelocity(AngularVelocity velocity){
        kickerMotor.set(TalonSRXControlMode.PercentOutput, velocity.in(RotationsPerSecond) / 100);
    }
    public void stopKicker(){
        kickerMotor.set(TalonSRXControlMode.PercentOutput, 0);
    }
}