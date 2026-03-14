package frc.robot.subsystems.shooter;

import frc.robot.Constants;
import frc.robot.state.KickerState;
import frc.robot.state.RebuiltStateMachine;
import frc.robot.state.ShooterState;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;

import static edu.wpi.first.units.Units.*;

public class Shooter extends SubsystemBase{
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private AngularVelocity spinnerVelocitySetpoint = RotationsPerSecond.of(60);
    private AngularVelocity defaultSpinnerSpeed = RotationsPerSecond.of(50);
    private AngularVelocity kickerVelocitySetpoint = RotationsPerSecond.of(60);
    private AngularVelocity spinnerVelocityOffset = RotationsPerSecond.of(0);
    
    private final Drive drive;
    
    private Command shooterCommand;

    public Shooter(ShooterIO io, RebuiltStateMachine stateMachine, Drive drive){
        this.io = io;
        this.drive = drive;

        stateMachine
                .state(ShooterState.STOPPED)
                .to(ShooterState.SPINNING_UP_ADAPTIVE)
                .run(setShooterCommandAdaptive());
        stateMachine
                .state(ShooterState.STOPPED)
                .to(ShooterState.SPINNING_UP_FIXED)
                .run(setShooterCommandFixed());

        stateMachine
                .state(ShooterState.SPINNING_UP_FIXED)
                .to(ShooterState.STOPPED)
                .run(cancelShooterCommand());
        stateMachine
                .state(ShooterState.SPINNING_UP_ADAPTIVE)
                .to(ShooterState.STOPPED)
                .run(cancelShooterCommand());
        stateMachine
                .state(ShooterState.AT_SPEED)
                .to(ShooterState.STOPPED)
                .run(cancelShooterCommand());

        stateMachine
                .state(ShooterState.SPINNING_UP_ADAPTIVE)
                .to(ShooterState.AT_SPEED)
                .transitionWhen(() -> Math.abs(getSpinnerClosedLoopError()) < 1);
        stateMachine
                .state(ShooterState.SPINNING_UP_FIXED)
                .to(ShooterState.AT_SPEED)
                .transitionWhen(() -> Math.abs(getSpinnerClosedLoopError()) < 1);

        stateMachine
                .state(KickerState.STOPPED)
                .to(KickerState.RUNNING)
                .run(runKicker());
        stateMachine
                .state(KickerState.REVERSING)
                .to(KickerState.RUNNING)
                .run(runKicker());

        stateMachine
                .state(KickerState.STOPPED)
                .to(KickerState.REVERSING)
                .run(reverseKicker());
        stateMachine
                .state(KickerState.RUNNING)
                .to(KickerState.REVERSING)
                .run(reverseKicker());

        stateMachine
                .state(KickerState.RUNNING)
                .to(KickerState.STOPPED)
                .run(stopKicker());
        stateMachine
                .state(KickerState.REVERSING)
                .to(KickerState.STOPPED)
                .run(stopKicker());
    }

    @Override
    public void periodic(){
        // Log key variables
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
        Logger.recordOutput("Kicker Velocity Setpoint", kickerVelocitySetpoint);
    }
    
    private Command setShooterCommandAdaptive() {
        this.shooterCommand = runSpinnerAdaptive(drive);
        return this.shooterCommand;
    }

    private Command setShooterCommandFixed() {
        this.shooterCommand = runSpinner();
        return this.shooterCommand;
    }

    private Command cancelShooterCommand() {
        return Commands.runOnce(() -> CommandScheduler.getInstance().cancelAll());
    }

    public AngularVelocity getIdealSpeed(Pose2d targetPose){ // Target pose is relevant to robot pose, x and y are delta x and delta y
        // TODO replace target pose withs state machine based identification and logic for aiming
        Distance distance;

        distance = Meters.of(Math.sqrt(
            Math.pow(targetPose.getX(), 2) +
            Math.pow(targetPose.getY(), 2)));

        SmartDashboard.putNumber("Distance From Hub (Meters)", distance.magnitude());
        double x = distance.in(Meters); 

        double targetVelocity = Math.min(80, Math.pow(x, 2) * 0.893293 + (1.75793 * x) + 40.677) + (spinnerVelocityOffset.magnitude());
        Logger.recordOutput("Spinner Velocity Setpoint", spinnerVelocitySetpoint);
        return RotationsPerSecond.of(targetVelocity); // TODO This function is guesswork and estimation
    }

    //TODO move this to be implemented in the runSpinner command, reference intake.java for example
    public void setSpinnerVelocitySetpoint(AngularVelocity speed){
        spinnerVelocitySetpoint = speed.plus(spinnerVelocityOffset);
    }
    
    public void stepSpinnerVelocitySetpoint(AngularVelocity speed){
        spinnerVelocityOffset = spinnerVelocityOffset.plus(speed);
    }

    public Command runSpinnerAdaptive(Drive drive){
        AngularVelocity idealSpeed = this.getIdealSpeed(drive.findShootingPose(drive.getPose()));
        double idealSpeedInRPS = idealSpeed.in(RotationsPerSecond);
        if (!Double.isNaN(idealSpeedInRPS)) {
            return Commands.runEnd(() -> this.io.setSpinnerVelocity(idealSpeed), this.io::stopSpinner);
        }
        return Commands.none();
    }

    public Command runSpinner(){
        return Commands.runEnd(() -> this.io.setSpinnerVelocity(defaultSpinnerSpeed.plus(spinnerVelocityOffset)), () -> this.io.stopSpinner());
    }
    public Command stopSpinner(){
        return Commands.runOnce(() -> this.io.stopSpinner());
    }

    public Command runKicker(){
        return Commands.runOnce(() -> this.io.setKickerVelocity(kickerVelocitySetpoint));
    }
    public Command reverseKicker(){
        return Commands.runOnce(() -> this.io.setKickerVelocity(kickerVelocitySetpoint.times(-0.25)));
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

    public Double getSpinnerClosedLoopError(){
        return inputs.leftSpinnerClosedLoopError;
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
