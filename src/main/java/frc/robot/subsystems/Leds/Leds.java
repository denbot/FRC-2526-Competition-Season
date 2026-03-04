package frc.robot.subsystems.Leds;

import static edu.wpi.first.units.Units.Seconds;

import bot.den.foxflow.RobotState;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.state.HubState;
import frc.robot.state.MatchState;
import frc.robot.state.RebuiltStateMachine;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;


public class Leds extends SubsystemBase{
	private int numLeds = 23;
	private AddressableLED led;
	private AddressableLEDBuffer ledBuffer;
//	private Limelights limelights;
	private CommandXboxController controller;
	private Shooter shooter;
	private Drive drive;
	private AddressableLEDBufferView leftHalf;
	private AddressableLEDBufferView rightHalf;
	private RebuiltStateMachine stateMachine;
	private final Timer timeUntilTransition = new Timer();
	private Time targetWaitTime = Seconds.zero();
	private Boolean isBlueActive = false;

	public Leds(CommandXboxController controller, Shooter shooter, Drive drive, RebuiltStateMachine stateMachine){
		this.led = new AddressableLED(0);
		this.ledBuffer = new AddressableLEDBuffer(numLeds);

		this.leftHalf = this.ledBuffer.createView(0, numLeds/2);
		this.rightHalf = this.ledBuffer.createView(numLeds/2, numLeds - 1);

		this.led.setLength(this.ledBuffer.getLength());
		this.led.setData(ledBuffer);
		this.led.start();

		this.controller = controller;
//		this.limelights = limelights;
		this.shooter = shooter;
		this.drive = drive;
		this.stateMachine = stateMachine;

		this.stateMachine.state(HubState.ACTIVE).to(HubState.INACTIVE).run(Commands.runOnce(() -> this.isBlueActive = drive.isBlue() ? false : true));
		this.stateMachine.state(HubState.INACTIVE).to(HubState.ACTIVE).run(Commands.runOnce(() -> this.isBlueActive = drive.isBlue() ? true : false));

		this.stateMachine.state(RobotState.TELEOP, MatchState.NONE).to(MatchState.TRANSITION_SHIFT).run(getMatchStateTimerCommand(MatchState.TRANSITION_SHIFT.timeInState, 3.0));
		this.stateMachine.state(MatchState.TRANSITION_SHIFT).to(MatchState.SHIFT_1).run(getMatchStateTimerCommand(MatchState.SHIFT_1.timeInState, 3.0));
		this.stateMachine.state(MatchState.SHIFT_1).to(MatchState.SHIFT_2).run(getMatchStateTimerCommand(MatchState.SHIFT_2.timeInState, 3.0));
		this.stateMachine.state(MatchState.SHIFT_2).to(MatchState.SHIFT_3).run(getMatchStateTimerCommand(MatchState.SHIFT_3.timeInState, 3.0));
		this.stateMachine.state(MatchState.SHIFT_3).to(MatchState.SHIFT_4).run(getMatchStateTimerCommand(MatchState.SHIFT_4.timeInState, 3.0));
		this.stateMachine.state(MatchState.SHIFT_4).to(MatchState.END_GAME).run(getMatchStateTimerCommand(MatchState.END_GAME.timeInState, 3.0));
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
		if(this.timeUntilTransition.hasElapsed(targetWaitTime)){
			CommandScheduler.getInstance().schedule(Commands.repeatingSequence(
				Commands.runOnce(() ->LEDPattern.solid(this.isBlueActive ? Color.kBlue : Color.kRed).applyTo(this.ledBuffer)),
				Commands.waitSeconds(0.5),
				Commands.runOnce(() ->LEDPattern.solid(Color.kWhite).applyTo(this.ledBuffer)))
				.withTimeout(Seconds.of(3)));
			this.timeUntilTransition.reset();
			this.timeUntilTransition.stop();			
		} 
		else if (this.controller.rightBumper().getAsBoolean() == true){

			LEDPattern baseLeft = LEDPattern.solid(Color.kGreen); // Base color for shooter speed
			LEDPattern baseRight = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kYellow, Color.kPurple); // Base color for auto aim

			// Blink green if spinner is at speed, blink red if not
			if (shooter.getSpinnerClosedLoopError() > 1) baseLeft = LEDPattern.solid(Color.kRed);
			baseLeft.blink(Seconds.of(0.5)).applyTo(leftHalf);
			
			// Get a value between 0-90 degrees (clamped by min.max) for how far off the dirve base is from aiming at the hub 
			double degreesOff = Math.min(Math.abs(drive.findAngleForShooting(drive.getPose()).getDegrees() - drive.getPose().getRotation().getDegrees()), 90);
			
			// Apply a mask to the graident from 0-1 for how close the drive base is to aiming fully
			baseRight.mask(LEDPattern.progressMaskLayer(() -> (90 - degreesOff) / 90)).applyTo(this.rightHalf);
		}
		// Limelights
//		else if (limelights.getTotalTagCount() >= 3) LEDPattern.solid(Color.kGreen).applyTo(this.ledBuffer);
		else LEDPattern.solid(Color.kRed).applyTo(this.ledBuffer);;

		this.led.setData(this.ledBuffer);
	}
}
