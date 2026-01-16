package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Intake extends SubsystemBase{
    private TalonSRX intakeMotor = new TalonSRX(5);

    public Command getSpinIntakeCommand(double percent){
        return Commands.runOnce(() -> intakeMotor.set(TalonSRXControlMode.PercentOutput, percent));
    }

    public Command getStopIntakeCommand(){
        return Commands.runOnce(() -> intakeMotor.set(TalonSRXControlMode.PercentOutput, 0));
    }
}