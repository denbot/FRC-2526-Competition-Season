package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.Limelights;

public class Leds extends SubsystemBase{
    private int numLeds = 10; // Test number, currently unknown
    private AddressableLED led;
    private AddressableLEDBuffer ledBuffer;
    private Limelights limelights;

    public Leds(Limelights limelights){
        this.led = new AddressableLED(0);
        this.ledBuffer = new AddressableLEDBuffer(numLeds);
        this.led.setLength(this.ledBuffer.getLength());

        this.led.setData(ledBuffer);
        this.led.start();
        this.limelights = limelights;
        
}
public void setPattern(LEDPattern pattern){
  pattern.applyTo(this.ledBuffer);
}
    @Override
  public void periodic(){
    this.led.setData(this.ledBuffer);
    if (limelights.geTotalTagCount() >= 3){
      LEDPattern.solid(Color.kGreen).applyTo(this.ledBuffer);

    }
    else{
    LEDPattern.solid(Color.kRed).applyTo(this.ledBuffer);
    }
}
}
