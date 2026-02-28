package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Degrees;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase{
    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    private Angle climberMotorPositionSetpoint = Degrees.of(30);

    public Climber(ClimberIO io){
        this.io = io;
    }

    @Override
    public void periodic(){
        // Log key variables
        io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);
        Logger.recordOutput("Climber Position Setpoint", climberMotorPositionSetpoint);
    }

    public Command setClimberMaxExtension(){
        return Commands.runOnce(() ->this.io.setClimberMaxExtension());
    }   
    
    public Command setClimberMinExtension(){
        return Commands.runOnce(() ->this.io.setClimberMinExtension());
    }

    public Command setClimberToSetpoint(Distance setpoint){
        return Commands.runOnce(() ->this.io.setClimberToSetpoint(setpoint));
    }

    public Boolean getClimberMotorConnected(){
        return inputs.climberMotorConnected;
    }

    public Angle getClimberMotorPosition(){
        return inputs.climberMotorPosition;
    }

    public double getClimberMotorClosedLoopError(){
        return inputs.climberMotorClosedLoopError;
    }

    public Current getClimberMotorCurrent(){
        return inputs.climberMotorAmps;
    }
}
