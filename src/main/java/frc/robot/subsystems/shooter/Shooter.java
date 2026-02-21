package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;

public class Shooter extends SubsystemBase{
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private AngularVelocity spinnerVelocitySetpoint = RotationsPerSecond.of(60);
    private AngularVelocity defaultSpinnerSpeed = RotationsPerSecond.of(60);
    private AngularVelocity kickerVelocitySetpoint = RotationsPerSecond.of(60);
    private AngularVelocity spinnerVelocityOffset = RotationsPerSecond.of(0);

    public Shooter(ShooterIO io){
        this.io = io;
    }

    @Override
    public void periodic(){
        // Log key variables
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
        Logger.recordOutput("Spinner Velocity Setpoint", spinnerVelocitySetpoint);
        Logger.recordOutput("Kicker Velocity Setpoint", kickerVelocitySetpoint);
    }

    public AngularVelocity getIdealSpeed(Pose2d robotPose, Pose2d targetPose){
        // TODO replace target pose withs state machine based identification and logic for aiming
        Distance distance;

        distance = Meters.of(Math.sqrt(
            Math.pow(robotPose.getX() - targetPose.getX(), 2) +
            Math.pow(robotPose.getY() - targetPose.getY(), 2)));

        System.out.println("Distance From Hub: " + distance.magnitude() + " meters");

        double x = distance.in(Meters);
        return RotationsPerSecond.of((Math.pow(x, 2) / 6) + (2.5 * x) + 39.3).plus(spinnerVelocityOffset); // TODO This function is guesswork and estimation
    }

    //TODO move this to be implemented in the runSpinner command, refference intake.java for example
    public void setSpinnerVelocitySetpoint(AngularVelocity speed){
        spinnerVelocitySetpoint = speed.plus(spinnerVelocityOffset);
    }
    
    public void stepSpinnerVelocitySetpoint(AngularVelocity speed){
        spinnerVelocityOffset = spinnerVelocityOffset.plus(speed);
    }

    public Command runSpinnerAddaptive(Drive drive, Pose2d targetPose){
        return Commands.runEnd(() -> this.io.setSpinnerVelocity(this.getIdealSpeed(drive.getPose(), targetPose)), () -> this.io.stopSpinner());
    }

    public Command runSpinner(){
        return Commands.runEnd(() -> this.io.setSpinnerVelocity(defaultSpinnerSpeed.plus(spinnerVelocityOffset)), () -> this.io.stopSpinner());
    }
    public Command stopSpinner(){
        return Commands.runOnce(() -> this.io.stopSpinner());
    }

    public Command runKicker(){
        return Commands.runEnd(() -> this.io.setKickerVelocity(kickerVelocitySetpoint), () -> this.io.stopKicker());
    }
    public Command reverseKicker(){
        return Commands.runEnd(() -> this.io.setKickerVelocity(kickerVelocitySetpoint.times(-0.25)), () -> this.io.stopKicker());
    }
    public Command stopKicker(){
        return Commands.runOnce(() -> this.io.stopKicker());
    }

    // Getters for private IO Variables    
    public AngularVelocity getSpinnerVelocitySetpoint(){
        return spinnerVelocitySetpoint;
    }

    public boolean getLeftSpinnerConnected(){
        return inputs.leftSpinnerMotorConnected;
    }
    public boolean getRightSpinnerConnected(){
        return inputs.rightSpinnerMotorConnected;
    }
    public boolean getKickerConnected(){
        return inputs.kickerMotorConnected;
    }

    public AngularVelocity getLeftSpinnerVelocity(){
        return inputs.leftSpinnerRotationSpeed;
    }
    public AngularVelocity getRightSpinnerVelocity(){
        return inputs.rightSpinnerRotationSpeed;
    }
    public AngularVelocity getkickerVelocity(){
        return inputs.kickerRotationSpeed;
    }

    public Angle getLeftSpinnerPosition(){
        return inputs.leftSpinnerPositionRots;
    }
    public Angle getRightSpinnerPosition(){
        return inputs.rightSpinnerPositionRots;
    }
    public Angle getkickerPosition(){
        return inputs.kickerPositionRots;
    }

    public Double getLeftSpinnerClosedLoopError(){
        return inputs.leftSpinnerClosedLoopError;
    }
    public Double getRightSpinnerClosedLoopError(){
        return inputs.rightSpinnerClosedLoopError;
    }
    public Double getKickerClosedLoopError(){
        return inputs.kickerClosedLoopError;
    }

    public Current getLeftSpinnerCurrent(){
        return inputs.leftSpinnerCurrentAmps;
    }
    public Current getRightSpinnerCurrent(){
        return inputs.rightSpinnerCurrentAmps;
    }
    public Current getKickerCurrent(){
        return inputs.kickerCurrentAmps;
    }
}   
