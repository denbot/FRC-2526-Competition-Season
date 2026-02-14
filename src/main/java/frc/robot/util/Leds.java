package frc.robot.util;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class Leds {
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
}