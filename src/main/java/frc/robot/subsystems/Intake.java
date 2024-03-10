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

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;

public class Intake extends SubsystemBase {
  private CANSparkMax intakeMotor;
  /** Creates a new Intake. */
  public Intake() {
intakeMotor = new CANSparkMax(9, MotorType.kBrushless);
intakeConfig();


  }
  public void stop(){
  intakeMotor.set(0.0);
  }

  public void intake(){
  intakeMotor.set(1.0);
  }

  public void   outake(){ 
intakeMotor.set(-0.5);
  }


 public void manualIntake(double speed){
  intakeMotor.set(speed);
 }


  public void intakeConfig(){
  intakeMotor.restoreFactoryDefaults();
  intakeMotor.enableVoltageCompensation(12);
  intakeMotor.setSmartCurrentLimit(30);
  intakeMotor.setInverted(false);
  intakeMotor.setIdleMode(IdleMode.kCoast);
  intakeMotor.setClosedLoopRampRate(0.15);
  intakeMotor.setOpenLoopRampRate(0.15);
  CANSparkMaxUtil.setCANSparkMaxBusUsage(intakeMotor, Usage.kMinimal, false);
  Timer.delay(0.2);
  intakeMotor.burnFlash();
  Timer.delay(0.2);
    }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("intake output duty", intakeMotor.getAppliedOutput());
    SmartDashboard.putNumber("intake output amps", intakeMotor.getOutputCurrent());
    SmartDashboard.putNumber("intake bus voltage", intakeMotor.getBusVoltage());
    // This method will be called once per scheduler run
  }
}
