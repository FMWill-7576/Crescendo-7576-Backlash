// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VictorClimber extends SubsystemBase {
  private VictorSPX leftClimb;
  private VictorSPX rightClimb;
   // private Compressor pcmCompressor;
   // Solenoid exampleSolenoidPCM;
  /** Creates a new Climber. */
  public VictorClimber() {
    leftClimb = new VictorSPX(21);
    rightClimb = new VictorSPX(20);
    climbConfig();
    //pcmCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
    //pcmCompressor.enableDigital();
    //pcmCompressor.disable();

   // exampleSolenoidPCM = new Solenoid(PneumaticsModuleType.CTREPCM, 1);

    //boolean enabled = pcmCompressor.isEnabled();
    //boolean pressureSwitch = pcmCompressor.getPressureSwitchValue();
    //double current = pcmCompressor.getCurrent();
  }

  public void stop(){
    leftClimb.set(VictorSPXControlMode.PercentOutput, 0.0);
    rightClimb.set(VictorSPXControlMode.PercentOutput, 0.0);
  }
  public void   manualclimb(double percentage){
     leftClimb.set(
      VictorSPXControlMode.PercentOutput,
     percentage
    );
  rightClimb.set(
    VictorSPXControlMode.PercentOutput, 
   - percentage
    ); 
  } 

  public void   outake(){ 
    leftClimb.set(
      VictorSPXControlMode.PercentOutput, 
      1.0
      );
    rightClimb.set(
      VictorSPXControlMode.PercentOutput, 
      1.0);
  }

  public void   drop(){ 
    leftClimb.set(
      VictorSPXControlMode.PercentOutput,
     0.35);
    rightClimb.set(
      VictorSPXControlMode.PercentOutput,
       0.30);
  }


  public void  strongHold(){ 
   /*  leftClimb.set(
      VictorSPXControlMode.PercentOutput,
     -0.35);
    rightClimb.set(
      VictorSPXControlMode.PercentOutput,
     -0.3);*/
  }
  public void   hold(){
    leftClimb.set(
      VictorSPXControlMode.PercentOutput,
    -0.27
    );
  /*  rightClimb.set(
    VictorSPXControlMode.PercentOutput, -0.22
    );*/
  }

  public void climbConfig(){
leftClimb.configFactoryDefault();
leftClimb.enableVoltageCompensation(true);
leftClimb.configVoltageCompSaturation(12.0);
leftClimb.setInverted(true);
leftClimb.setNeutralMode(NeutralMode.Brake);


rightClimb.configFactoryDefault();
rightClimb.enableVoltageCompensation(true);
rightClimb.configVoltageCompSaturation(12.0);
rightClimb.setInverted(true);
rightClimb.setNeutralMode(NeutralMode.Brake);
}

  /* public void pistonTest(){
    exampleSolenoidPCM.toggle();
  } */

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
