package frc.robot.subsystems.Leds;

import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;


import bot.den.foxflow.RobotState;
import bot.den.foxflow.StateMachine;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.control.OperatorController;
import frc.robot.RobotContainer;
import frc.robot.state.HubState;
import frc.robot.state.MatchState;
import frc.robot.state.RebuiltStateMachine;
import frc.robot.state.ShooterState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Limelights;


public class Leds extends SubsystemBase{
	private int numLeds = 46;
	private AddressableLED led;
	private AddressableLEDBuffer ledBuffer;
	private Limelights limelights;
	private OperatorController controller;
	private Shooter shooter;
	private Drive drive;
	private AddressableLEDBufferView leftHalf;
	private AddressableLEDBufferView rightHalf;
	private AddressableLEDBufferView leftLimelight;
	private AddressableLEDBufferView frontLimelight;
	private AddressableLEDBufferView rightLimelight;
	private RebuiltStateMachine stateMachine;
	private final Timer timeUntilTransition = new Timer();
	private Time targetWaitTime = Seconds.zero();
	private Boolean isBlueActive = false;

	public Leds(Limelights limelights, OperatorController controller, Shooter shooter, Drive drive, RebuiltStateMachine stateMachine){
		this.led = new AddressableLED(0);
		this.ledBuffer = new AddressableLEDBuffer(numLeds);

		this.leftHalf = this.ledBuffer.createView(4, numLeds/2);
		this.rightHalf = this.ledBuffer.createView(numLeds/2+1, numLeds - 4);
		this.rightLimelight = this.ledBuffer.createView(0, 4);
		this.frontLimelight = this.ledBuffer.createView(numLeds/2-2, numLeds/2+1);
		this.leftLimelight = this.ledBuffer.createView(numLeds - 5, numLeds - 1);

		this.led.setLength(this.ledBuffer.getLength());
		this.led.setData(ledBuffer);
		this.led.start();

		this.controller = controller;
		this.limelights = limelights;
		this.shooter = shooter;
		this.drive = drive;
		this.stateMachine = stateMachine;

		this.stateMachine.state(HubState.ACTIVE).to(HubState.INACTIVE).run(Commands.runOnce(() -> this.isBlueActive = RobotContainer.isBlue() ? false : true));
		this.stateMachine.state(HubState.INACTIVE).to(HubState.ACTIVE).run(Commands.runOnce(() -> this.isBlueActive = RobotContainer.isBlue() ? true : false));

		this.stateMachine.state(RobotState.TELEOP, MatchState.NONE).to(MatchState.TRANSITION_SHIFT).run(getMatchStateTimerCommand(MatchState.TRANSITION_SHIFT.timeInState, 3.0));
		this.stateMachine.state(MatchState.TRANSITION_SHIFT).to(MatchState.SHIFT_1).run(getMatchStateTimerCommand(MatchState.SHIFT_1.timeInState, 5.0));
		this.stateMachine.state(MatchState.SHIFT_1).to(MatchState.SHIFT_2).run(getMatchStateTimerCommand(MatchState.SHIFT_2.timeInState, 7.0));
		this.stateMachine.state(MatchState.SHIFT_2).to(MatchState.SHIFT_3).run(getMatchStateTimerCommand(MatchState.SHIFT_3.timeInState, 7.0));
		this.stateMachine.state(MatchState.SHIFT_3).to(MatchState.SHIFT_4).run(getMatchStateTimerCommand(MatchState.SHIFT_4.timeInState, 7.0));
		this.stateMachine.state(MatchState.SHIFT_4).to(MatchState.END_GAME).run(getMatchStateTimerCommand(MatchState.END_GAME.timeInState, 7.0));
	}

	public Command getMatchStateTimerCommand(Time timeInState, double offset){
		return Commands.runOnce(()-> {
			
				this.targetWaitTime = timeInState.minus(Seconds.of(offset));
				this.timeUntilTransition.restart();
			});
	}

	public void setPattern(LEDPattern pattern){
		pattern.applyTo(this.ledBuffer);
	}
	
	@Override
	public void periodic(){
		if (stateMachine.currentState().robotState() == RobotState.AUTO) {
			LEDPattern baseColor = LEDPattern.gradient(GradientType.kContinuous, Color.kOrangeRed, Color.kBlack, Color.kWhite);
			baseColor.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), Meters.of(0.02)).applyTo(this.ledBuffer);
		} else if(
			this.timeUntilTransition.hasElapsed(targetWaitTime) && 
			stateMachine.currentState().robotState() == RobotState.TELEOP) {
			LEDPattern.solid(
				timeUntilTransition.get() % 0.5 > 0.25 ?
				Color.kCyan :
				Color.kOrange
			).applyTo(this.ledBuffer);	
		} else {
			if (DriverStation.getAlliance().isPresent()) {
				if(RobotContainer.isBlue()) {
					LEDPattern
						.gradient(GradientType.kContinuous, Color.kBlue, Color.kOrangeRed)
						.scrollAtAbsoluteSpeed(MetersPerSecond.of(0.5), Meters.of(0.02))
						.applyTo(this.ledBuffer);
				} else {
					LEDPattern
					.gradient(GradientType.kContinuous, Color.kRed, Color.kOrangeRed)
					.scrollAtAbsoluteSpeed(MetersPerSecond.of(0.5), Meters.of(0.02))
					.applyTo(this.ledBuffer);
				}
			} else {
				LEDPattern.solid(Color.kOrangeRed).applyTo(this.ledBuffer);
			} 
		}

		if (stateMachine.currentState().shooterState() != ShooterState.STOPPED){
			LEDPattern baseColor = LEDPattern.gradient(GradientType.kContinuous, Color.kBlue, Color.kViolet);
			if (stateMachine.currentState().shooterState() == ShooterState.AT_SPEED) 
			{
				baseColor = LEDPattern.gradient(GradientType.kContinuous, Color.kRed, Color.kOrangeRed, Color.kOrange);
			}
			baseColor.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), Meters.of(0.02)).applyTo(this.ledBuffer);
			
		}
		// Limelights
		if (limelights.getBackLeftTags() > 0) {
			LEDPattern.solid(Color.kGreen).applyTo(this.leftLimelight);
		} else {
			LEDPattern.solid(Color.kBlack).applyTo(this.leftLimelight);
		}
		if (limelights.getFrontTags() > 0) {
			LEDPattern.solid(Color.kGreen).applyTo(this.frontLimelight);
		} else {
			LEDPattern.solid(Color.kBlack).applyTo(this.frontLimelight);
		}
		if (limelights.getBackRightTags() > 0) {
			LEDPattern.solid(Color.kGreen).applyTo(this.rightLimelight);
		} else {
			LEDPattern.solid(Color.kBlack).applyTo(this.rightLimelight);
		}
		
		if (controller.enableButtonBoxSwitch.getAsBoolean()) {
			LEDPattern
			.rainbow(255, 255)
			.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), Meters.of(0.02)).applyTo(this.ledBuffer);
		} 

		this.led.setData(this.ledBuffer);
	}
}
