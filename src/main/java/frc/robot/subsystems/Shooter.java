// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;


public class Shooter extends SubsystemBase {
  private CANSparkMax shootMotorLeft;
  private CANSparkMax shootMotorRight;
  private RelativeEncoder integratedLeftEncoder;
  private final SparkPIDController shootController ;
  /** Creates a new shootator. */
  public Shooter() {
    shootMotorLeft= new CANSparkMax(12, MotorType.kBrushless);
    shootMotorRight= new CANSparkMax(13, MotorType.kBrushless);
    shootController = shootMotorLeft.getPIDController();
    integratedLeftEncoder = shootMotorLeft.getEncoder();
    shooterConfig();
  
  }

  public void shooterConfig() { 
    shootMotorLeft.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(shootMotorLeft, Usage.kVelocityOnly,true);
    shootMotorLeft.setSmartCurrentLimit(40);
    shootMotorLeft.setInverted(false);
    shootMotorLeft.setIdleMode(IdleMode.kBrake);
    integratedLeftEncoder.setPositionConversionFactor(1.0); 
    integratedLeftEncoder.setPosition(0.0);
    integratedLeftEncoder.setVelocityConversionFactor(1.0);  
    shootController.setP(0.0);
    shootController.setI(0.0);
    shootController.setD(0.0);
    shootController.setIMaxAccum(0.0,0);
    shootController.setIZone(0.0);
    shootMotorLeft.enableVoltageCompensation(12.0);
    shootController.setOutputRange(-0.48, 0.48);
    //integratedArmEncoder.setReverseDirection(false); // bu
    Timer.delay(0.2);
    shootMotorLeft.burnFlash();
    Timer.delay(0.2);
}




  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
