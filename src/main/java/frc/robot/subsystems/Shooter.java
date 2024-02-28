// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;


public class Shooter extends SubsystemBase {
  private CANSparkMax shootMotorUp;
  private CANSparkMax shootMotorDown;
  private RelativeEncoder integratedUpEncoder;
  private Encoder shooterBore;
  private RelativeEncoder shooterRelative;
  private final SparkPIDController shootController ;
  private PIDController shooterPID;
  private double setpoint;
  /** Creates a new shootator. */
  public Shooter() {
    shootMotorUp= new CANSparkMax(12, MotorType.kBrushless);
    shootMotorDown= new CANSparkMax(13, MotorType.kBrushless);
    shootController = shootMotorUp.getPIDController();
    shooterPID = new PIDController(0, 0, 0);
    integratedUpEncoder = shootMotorUp.getEncoder();
    //shooterBore = new Encoder(0, 0, false, EncodingType.k1X);
    shooterRelative = shootMotorUp.getAlternateEncoder(2048);
    shooterConfig();
  
  }

  public void shooterConfig() { 
    shootMotorUp.restoreFactoryDefaults();
    shootMotorDown.restoreFactoryDefaults();
    shootController.setFeedbackDevice(shooterRelative);
    CANSparkMaxUtil.setCANSparkMaxBusUsage(shootMotorUp, Usage.kVelocityOnly,true);
    CANSparkMaxUtil.setCANSparkMaxBusUsage(shootMotorDown, Usage.kVelocityOnly,true);
    shootMotorUp.setSmartCurrentLimit(45);
    shootMotorDown.setSmartCurrentLimit(45);
    shootMotorUp.setInverted(false);
    shootMotorUp.setIdleMode(IdleMode.kCoast);
    shootMotorUp.setIdleMode(IdleMode.kCoast);
    integratedUpEncoder.setPositionConversionFactor(1.0); 
    integratedUpEncoder.setPosition(0.0);
    integratedUpEncoder.setVelocityConversionFactor(1.0);  
    shootController.setP(0.0);
    shootController.setI(0.0);
    shootController.setD(0.0);
    shootController.setFF(0.0);
    shootController.setIMaxAccum(0.0,0);
    shootController.setIZone(0.0);
    shootMotorUp.enableVoltageCompensation(12.0);
    shootController.setOutputRange(-1.0, 1.0);
    //integratedArmEncoder.setReverseDirection(false); // bu
    shootMotorDown.follow(shootMotorUp);
    Timer.delay(0.2);
    shootMotorUp.burnFlash();
    Timer.delay(0.2);
    shootMotorDown.burnFlash();
    Timer.delay(0.2);
}

public void shootDriveManual(double percentage){
shootMotorUp.set(percentage);
}

public void shooterSet(double RPM) {

        shootController.setReference(
         RPM,
         CANSparkMax.ControlType.kVelocity,
         0
        );
         
       setpoint=RPM;
      }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("shooter rpm", shooterRelative.getVelocity());
    SmartDashboard.putNumber("shooter applied duty", shootMotorUp.getAppliedOutput());
  }
}
