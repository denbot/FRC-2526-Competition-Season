package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Limelights;


public class Leds extends SubsystemBase{
    private int numLeds = 10; // Test number, currently unknown
    private AddressableLED led;
    private AddressableLEDBuffer ledBuffer;
    private Limelights limelights;
    private CommandXboxController controller;
    private Shooter shooter;

    public Leds(Limelights limelights, CommandXboxController controller, Shooter shooter){
        this.led = new AddressableLED(0);
        this.ledBuffer = new AddressableLEDBuffer(numLeds);
        this.led.setLength(this.ledBuffer.getLength());
        this.controller = controller;
        this.led.setData(ledBuffer);
        this.led.start();
        this.limelights = limelights;
        this.shooter = shooter;
}
public void setPattern(LEDPattern pattern){
  pattern.applyTo(this.ledBuffer);
}
    @Override
  public void periodic(){
    this.led.setData(this.ledBuffer);
    if (this.controller.rightBumper().getAsBoolean() == true){
      if (shooter.getLeftSpinnerClosedLoopError() < 1){
    LEDPattern base = LEDPattern.solid(Color.kGreen);
    LEDPattern blinking = base.blink(Seconds.of(0.5));
    blinking.applyTo(this.ledBuffer);
      }
      else{
            LEDPattern base = LEDPattern.solid(Color.kRed);
    LEDPattern blinking = base.blink(Seconds.of(0.5));
    blinking.applyTo(this.ledBuffer);
      }
    }
    else if (limelights.geTotalTagCount() >= 3){
      LEDPattern.solid(Color.kGreen).applyTo(this.ledBuffer);

    }
    else{
    LEDPattern.solid(Color.kRed).applyTo(this.ledBuffer);
    }

}
}
