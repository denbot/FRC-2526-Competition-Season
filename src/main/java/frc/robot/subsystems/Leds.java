package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Limelights;


public class Leds extends SubsystemBase{
	private int numLeds = 10; // Test number, currently unknown
	private AddressableLED led;
	private AddressableLEDBuffer ledBuffer;
	private Limelights limelights;
	private CommandXboxController controller;
	private Shooter shooter;
	private Drive drive;
	private AddressableLEDBufferView leftHalf;
	private AddressableLEDBufferView rightHalf;

	public Leds(Limelights limelights, CommandXboxController controller, Shooter shooter, Drive drive){
		this.led = new AddressableLED(0);
		this.ledBuffer = new AddressableLEDBuffer(numLeds);
		this.leftHalf = this.ledBuffer.createView(0, numLeds/2);
		this.rightHalf = this.ledBuffer.createView(numLeds/2, numLeds);

		this.led.setLength(this.ledBuffer.getLength());
		this.led.setData(ledBuffer);
		this.led.start();

		this.controller = controller;
		this.limelights = limelights;
		this.shooter = shooter;
		this.drive = drive;
	}
	
	public void setPattern(LEDPattern pattern){
		pattern.applyTo(this.ledBuffer);
	}
	
	@Override
	public void periodic(){
		if (this.controller.rightBumper().getAsBoolean() == true){
			LEDPattern baseLeft = LEDPattern.solid(Color.kGreen); // Base color for shooter speed
			LEDPattern baseRight = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kYellow, Color.kPurple); // Base color for auto aim

			// Blink green if spinner is at speed, blink red if not
			if (shooter.getLeftSpinnerClosedLoopError() > 1) baseLeft = LEDPattern.solid(Color.kRed);
			baseLeft.blink(Seconds.of(0.5)).applyTo(leftHalf);
			
			// Get a value between 0-90 degrees (clamped by min.max) for how far off the dirve base is from aiming at the hub 
			double degreesOff = Math.max(Math.min(Math.abs(drive.findAngleForShooting(drive.getPose()).getDegrees() - drive.getPose().getRotation().getDegrees()), 90), 0);
			
			// Apply a mask to the graident from 0-1 for how close the drive base is to aiming fully
			baseRight.mask(LEDPattern.progressMaskLayer(() -> (90 - degreesOff) / 90)).applyTo(this.rightHalf);
		}
		// Limelights
		else if (limelights.geTotalTagCount() >= 3) LEDPattern.solid(Color.kGreen).applyTo(this.ledBuffer);
		else LEDPattern.solid(Color.kRed).applyTo(this.ledBuffer);;

		this.led.setData(this.ledBuffer);
	}
}
