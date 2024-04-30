// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;

public class Climber extends SubsystemBase {
  private CANSparkMax climbMotorLeft;
  private CANSparkMax climbMotorRight;

  /** Creates a new Climber. */
  public Climber() {
      climbMotorLeft  = new CANSparkMax(16, MotorType.kBrushless);
      climbMotorRight  = new CANSparkMax(17, MotorType.kBrushless); 
  climbMotorConfig();
  }


  private void climbMotorConfig(){
    climbMotorLeft.restoreFactoryDefaults();
    climbMotorRight.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(climbMotorLeft, Usage.kMinimal,true);
    CANSparkMaxUtil.setCANSparkMaxBusUsage(climbMotorRight, Usage.kMinimal,true);
    climbMotorLeft.setSmartCurrentLimit(60);
    climbMotorRight.setSmartCurrentLimit(60);
    climbMotorLeft.setInverted(false);
    climbMotorRight.setInverted(false);
    climbMotorLeft.setIdleMode(IdleMode.kBrake);
    climbMotorRight.setIdleMode(IdleMode.kBrake);
    climbMotorLeft.enableVoltageCompensation(12.0);
    climbMotorRight.enableVoltageCompensation(12.0);
    Timer.delay(0.2);
    climbMotorLeft.burnFlash();
    climbMotorRight.burnFlash();
    climbMotorLeft.setOpenLoopRampRate(0.1);
    climbMotorRight.setOpenLoopRampRate(0.1);
    Timer.delay(0.2);
   
  }

   public void climbUp(){
    climbMotorLeft.set(0.9);
    climbMotorRight.set(-0.9);

   }

public void manualClimb(double percentage){
  climbMotorLeft.set(percentage);
  climbMotorRight.set(-percentage);
}
   public void climbDown(){
    climbMotorLeft.set(-0.9);
    climbMotorRight.set(+0.9);

   }

   public void leftClimb(){
    climbMotorLeft.set(-0.5);

   }

   public void leftClimbUp(){
    climbMotorLeft.set(0.5);
   }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
