// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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
  private RelativeEncoder integratedDownEncoder;
  private Encoder shooterBore;
  private RelativeEncoder shooterRelativeBore;
  private final SparkPIDController shootController ;
   private final SparkPIDController shootControllerDown ;
  private PIDController shooterPID;
   private SimpleMotorFeedforward shooterFF;
  private double setpoint;
  private double downSetpoint;
  /** Creates a new shootator. */
  public Shooter() {
    shootMotorUp= new CANSparkMax(12, MotorType.kBrushless);
    shootMotorDown= new CANSparkMax(13, MotorType.kBrushless);
    shootController = shootMotorUp.getPIDController();
    shootControllerDown = shootMotorDown.getPIDController();
    shooterPID = new PIDController(0, 0, 0);
    shooterFF = new SimpleMotorFeedforward(0, 0);
    integratedUpEncoder = shootMotorUp.getEncoder();
    integratedDownEncoder = shootMotorDown.getEncoder();
    //shooterBore = new Encoder(0, 0, false, EncodingType.k1X);
    shooterRelativeBore = shootMotorUp.getAlternateEncoder(2048);
    setpoint=4000;
    shooterConfig();
  
  }

  public void shooterConfig() { 
    shootMotorUp.restoreFactoryDefaults();
    shootMotorDown.restoreFactoryDefaults();
   // shootController.setFeedbackDevice(shooterRelative);
    shootController.setFeedbackDevice(integratedUpEncoder);
    shootControllerDown.setFeedbackDevice(integratedDownEncoder);
    CANSparkMaxUtil.setCANSparkMaxBusUsage(shootMotorUp, Usage.kVelocityOnly,true);
    CANSparkMaxUtil.setCANSparkMaxBusUsage(shootMotorDown, Usage.kVelocityOnly,true);
    shootMotorUp.setSmartCurrentLimit(40);
    shootMotorDown.setSmartCurrentLimit(40);
    shootMotorUp.setInverted(false);
    shootMotorDown.setInverted(false);
    shootMotorUp.setIdleMode(IdleMode.kCoast);
    shootMotorDown.setIdleMode(IdleMode.kCoast);
    shooterRelativeBore.setVelocityConversionFactor(1.0);
    integratedUpEncoder.setPositionConversionFactor(1.0); 
    integratedUpEncoder.setPosition(0.0);
    integratedDownEncoder.setPosition(0.0);
    integratedUpEncoder.setVelocityConversionFactor(1.6);  
    integratedDownEncoder.setVelocityConversionFactor(1.6); 
    shootController.setP(0.0001);
    shootController.setI(0.0);
    shootController.setD(0.0);
    shootController.setFF(0.000118);
    shootController.setIMaxAccum(0.0,0);
    shootController.setIZone(0.0);
    shootControllerDown.setP(0.0001);
    shootControllerDown.setI(0.0);
    shootControllerDown.setD(0.0);
    shootControllerDown.setFF(0.000115);
    shootControllerDown.setIMaxAccum(0.0,0);
    shootControllerDown.setIZone(0.0);
    shootMotorUp.enableVoltageCompensation(12.0);
    shootMotorDown.enableVoltageCompensation(12.0);
    shootMotorUp.setClosedLoopRampRate(0.05);
    shootMotorDown.setClosedLoopRampRate(0.05);
    shootMotorUp.setOpenLoopRampRate(0.05);
    shootMotorDown.setOpenLoopRampRate(0.05);
    shootController.setOutputRange(-1.0, 1.0);
    shootControllerDown.setOutputRange(-1.0, 1.0);
    //integratedArmEncoder.setReverseDirection(false); // bu
    //shootMotorDown.follow(shootMotorUp);
    Timer.delay(0.2);
    shootMotorUp.burnFlash();
    Timer.delay(0.2);
    shootMotorDown.burnFlash();
    Timer.delay(0.2);
}

public void shootDriveManual(double percentage){
shootMotorUp.set(percentage);
shootMotorDown.set(percentage);
}


public Boolean isShooterAtSetpoint() {

   return  Math.abs((integratedUpEncoder.getVelocity()-setpoint)) <=200 ; 
  }


public void shooterSet(double RPM,double downRPM) {
  downSetpoint=downRPM;
  setpoint=RPM;

        shootController.setReference(
         setpoint,
         CANSparkMax.ControlType.kVelocity,
         0
        );


        shootControllerDown.setReference(
         downSetpoint,
         CANSparkMax.ControlType.kVelocity,
         0
        );
         
      }



 public void shooterIdle() {

        shootController.setReference(
         300,
         CANSparkMax.ControlType.kVelocity,
         0
        );


        shootControllerDown.setReference(
         300,
         CANSparkMax.ControlType.kVelocity,
         0
        );
         
      }
     
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("shooter measured rpm up BORE", shooterRelativeBore.getVelocity());
    SmartDashboard.putNumber("shooter measured rpm up", integratedUpEncoder.getVelocity());
    SmartDashboard.putNumber("shooter measured rpm down", integratedDownEncoder.getVelocity());
    SmartDashboard.putNumber("shooter setpoint rpm", setpoint);
    SmartDashboard.putNumber("shooter setpoint rpm down", downSetpoint);
    SmartDashboard.putNumber("shooter applied duty", shootMotorUp.getAppliedOutput());
    SmartDashboard.putNumber("shooter bus voltage", shootMotorUp.getBusVoltage());
    SmartDashboard.putNumber("shooter applied amp", shootMotorUp.getOutputCurrent());
  }


}
