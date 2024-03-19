// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class LedSubsystem extends SubsystemBase {
  private AddressableLED led;
  private AddressableLEDBuffer ledBuffer;
  private int m_rainbowFirstPixelHue;
  private int m_orangeFirstPixelHue;
  /** Creates a new LedSubsystem. */
  public LedSubsystem() {
    // PWM port 9
    // Must be a PWM header, not MXP or DIO
    led = new AddressableLED(0);

    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    ledBuffer = new AddressableLEDBuffer(300);
    led.setLength(ledBuffer.getLength());

    // Set the data
    led.setData(ledBuffer);
    led.start();

  }

  private void rainbow() {
    // For every pixel
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;
      // Set the value
      ledBuffer.setHSV(i, hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 3;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;
  }
  

  private void orange(){
        for (var i = 0; i < ledBuffer.getLength(); i++) {
      //final var hue = (m_rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;
      // Set the value
      ledBuffer.setRGB(i, 2000,3, 5);
    }
     led.setData(ledBuffer);
    //m_orangeFirstPixelHue +=3;
    //m_orangeFirstPixelHue %= 255;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
   // Fill the buffer with a rainbow
    orange();
    // Set the LEDs
    //led.setData(ledBuffer);
    }
  


public void disabledPeriodic() {
    // Fill the buffer with a rainbow
   rainbow();
    // Set the LEDs
   led.setData(ledBuffer);
  }
      
}
