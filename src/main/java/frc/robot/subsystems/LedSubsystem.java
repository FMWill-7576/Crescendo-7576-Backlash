// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class LedSubsystem extends SubsystemBase {
  private AddressableLED led;
  private AddressableLEDBuffer ledBuffer;
  private int m_rainbowFirstPixelHue;
  private int m_orangeFirstPixelHue;
  private int pulseOffset;
  private int pulseOffset2;
  Indexer s_Indexer;
  private int blueStreakLED = 0;
  private int numLoops = 0;
      int bluePulseBrightness = 0;
  public boolean shootMode;

  /** Creates a new LedSubsystem. */
  public LedSubsystem(Indexer s_Indexer) {
    // PWM port 0
    // Must be a PWM header, not MXP or DIO
    led = new AddressableLED(0);

    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    ledBuffer = new AddressableLEDBuffer(138);
    led.setLength(ledBuffer.getLength());

    // Set the data
    led.setData(ledBuffer);
    led.start();
    m_rainbowFirstPixelHue=0;
    this.s_Indexer = s_Indexer;
    pulseOffset = 0;
    pulseOffset2 = 0;
    

  }



  public void setShootMode(boolean shootMode){
    this.shootMode = shootMode;
  
  }



  private void rainbow() {
    
    // For every pixel
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final int hue = (m_rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;
      // Set the value
      ledBuffer.setHSV(i, hue, 255, 50);
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 3;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;
    led.setData(ledBuffer);
  }
  

  private void orange(){
      //Color turuncu = new Color("#FA9904");
    
        for (var i = 0; i < ledBuffer.getLength(); i++) {
      // Set the value
      ledBuffer.setHSV(i, 3, 255, 255);
    }
    led.setData(ledBuffer);
  }



    public void blue(){
      //Color turuncu = new Color("#FA9904");
    
        for (var i = 0; i < ledBuffer.getLength(); i++) {
      // Set the value
      ledBuffer.setRGB(i, 0, 0, 255);
    }
    led.setData(ledBuffer);
  }

    private void pink(){
      //Color turuncu = new Color("#FA9904");
    
        for (var i = 0; i < ledBuffer.getLength(); i++) {
      // Set the value
      ledBuffer.setRGB(i, 255, 0, 180);
    }
    led.setData(ledBuffer);
  }
    private void redDisabled(){
      //Color turuncu = new Color("#FA9904");
       // Fading is done by changing the value of the color over a sine wave
       double ledWavelength = (2 * Math.PI) / 120;
       double timeWavelength = (2 * Math.PI) / 5;
       double timeOffset = Timer.getFPGATimestamp() * timeWavelength;

       for (int i = 0; i < 71; i++) {
        double interpolation = Math.sin(ledWavelength * i + timeOffset) * 0.5 + 0.5;

        ledBuffer.setRGB(
                 i,
                interpolate(180, 255, interpolation),
                interpolate(0, 0, interpolation),
                interpolate(0, 0, interpolation)
        );
       }
        for (int n = 137; n > 70; n--) {
        double interpolation2 = -Math.sin(ledWavelength * n + timeOffset) * 0.5 + 0.5;

        ledBuffer.setRGB(
                 n,
                interpolate(180, 255, interpolation2),
                interpolate(20, 0, interpolation2),
                interpolate(0, 0, interpolation2)
        );
    }

    led.setData(ledBuffer);
  
  }

private void blueDisabled(){
      //Color turuncu = new Color("#FA9904");
       // Fading is done by changing the value of the color over a sine wave
       double ledWavelength = (2 * Math.PI) / 120;
       double timeWavelength = (2 * Math.PI) / 5;
       double timeOffset = Timer.getFPGATimestamp() * timeWavelength;

       for (int i = 0; i < 71; i++) {
        double interpolation = Math.sin(ledWavelength * i + timeOffset) * 0.5 + 0.5;

        ledBuffer.setRGB(
                 i,
                interpolate(0, 0, interpolation),
                interpolate(0, 90, interpolation),
                interpolate(255, 55, interpolation)
        );
       }
        for (int n = 137; n > 70; n--) {
        double interpolation2 = -Math.sin(ledWavelength * n + timeOffset) * 0.5 + 0.5;

        ledBuffer.setRGB(
                 n,
                interpolate(0, 0, interpolation2),
                interpolate(0, 90, interpolation2),
                interpolate(255, 55, interpolation2)
        );
    }

    led.setData(ledBuffer);
  
  }




  public void redPulse(){
    int intensity = 255 - pulseOffset;
    for (int i = 0; i < ledBuffer.getLength() / 2 + 1; i++) {
      ledBuffer.setRGB(i, (intensity + 25*i) % 255, 0, 0);
      ledBuffer.setRGB(ledBuffer.getLength()-1-i, (intensity + 25*i) % 255, 0, 0);
  }
  pulseOffset = (pulseOffset+10) % 255;
  led.setData(ledBuffer);
  }

 private void orangePulse(){
    int intensity = 255 - pulseOffset;
    for (int i = 0; i < ledBuffer.getLength() / 2 + 1; i++) {
      ledBuffer.setHSV(i, 3, 255, (intensity + 25*i) % 255);
      ledBuffer.setHSV(ledBuffer.getLength()-1-i, 3, 255, (intensity + 25*i) % 255);
  }
  pulseOffset = (pulseOffset+15) % 255;
  led.setData(ledBuffer);
  }



   public void bluePulse(){
    int intensity = 255 - pulseOffset2;
    for (int i = 0; i < ledBuffer.getLength() / 2 + 1; i++) {
      ledBuffer.setRGB(i, 0, 0, (intensity + 25*i) % 255);
      ledBuffer.setRGB(ledBuffer.getLength()-1-i, 0 ,0, (intensity + 25*i) % 255);
  }
  pulseOffset2 = (pulseOffset2+10) % 255;
  led.setData(ledBuffer);
  }

  public void blueFlash(){
    for (int n = 0; n < ledBuffer.getLength(); n++) {
      // Sets the specified LED to the RGB values for blue
      ledBuffer.setRGB(n, 0, 0, bluePulseBrightness);
       }
    
       //increase brightness
       bluePulseBrightness += 10;
    
       //Check bounds
       bluePulseBrightness %= 255;
    
       led.setData(ledBuffer);
    
      }
    





  public void blueStreak(){
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for blue
      ledBuffer.setRGB(i, 0, 0, 0);
      ledBuffer.setRGB(blueStreakLED, 0, 0, 255);
  
     // blueStreakLED += 2;
   }
   //increase brightness
   if (numLoops%3 == 0){
      blueStreakLED += 1;


      //Check bounds
      blueStreakLED %= ledBuffer.getLength();
      
    }

   led.setData(ledBuffer);


   numLoops += 1;
   //Timer.delay(0.2);
   

  }



  private int interpolate(int start, int end, double interpolationValue) {
    return (int) (start + (end - start) * interpolationValue);
}


 public Command LEDCommand(){
  return run(() -> {
      if (s_Indexer.isNoteInIndexer()) {
      if (shootMode){
        if(DriverStation.getAlliance().orElse(null)==Alliance.Blue){
        blueFlash();
      }
      else{
        redPulse();
      }
    }
     else{
    pink();
  }
}

    else {
      orangePulse();
    }
      }
    );
  }
  
 
  @Override
  public void periodic() {
    Alliance ally = DriverStation.getAlliance().orElse(null);
    // This method will be called once per scheduler run
   // Fill the buffer with a rainbow
   if (!DriverStation.isEnabled()) {
      if (ally != null) {
        if (ally== Alliance.Blue){
        blueDisabled();
        }
        else{
         redDisabled();
         
        } 
      }
      else {
      rainbow();
    }
  
    }
      
}
}