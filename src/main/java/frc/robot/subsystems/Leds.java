package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Leds extends SubsystemBase{
    private int numLeds = 10; // Test number, currently unknown
    private AddressableLED led;
    private AddressableLEDBuffer ledBuffer;

    public Leds(){
        this.led = new AddressableLED(0);
        this.ledBuffer = new AddressableLEDBuffer(numLeds);
        this.led.setLength(this.ledBuffer.getLength());

        this.led.setData(ledBuffer);
        this.led.start();
        
}
public void setPattern(LEDPattern pattern){
  pattern.applyTo(this.ledBuffer);
}
    @Override
  public void periodic(){
    this.led.setData(this.ledBuffer);
}
    }
