// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;

public class Indexer extends SubsystemBase {
  private CANSparkMax indexMotor;
  private static DigitalInput beamBreak;
  /** Creates a new index. */
  public Indexer() {
indexMotor = new CANSparkMax(12, MotorType.kBrushless);
beamBreak = new DigitalInput(0);

indexConfig(); 

  }
  public void stop(){
  indexMotor.set(0.0);
  }

  public void hold(){
  indexMotor.set(0.5);
  }

  public void  shoot(){ 
  indexMotor.set(-0.5);
  }
  
  public static boolean hasNote(){
    return beamBreak.get();
  }
 


  public void indexConfig(){
  indexMotor.restoreFactoryDefaults();
  indexMotor.enableVoltageCompensation(12);
  indexMotor.setSmartCurrentLimit(25);
  indexMotor.setInverted(false);
  indexMotor.setIdleMode(IdleMode.kCoast);
  CANSparkMaxUtil.setCANSparkMaxBusUsage(indexMotor, Usage.kMinimal, false);
  Timer.delay(0.2);
  indexMotor.burnFlash();
  Timer.delay(0.2);
    }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}