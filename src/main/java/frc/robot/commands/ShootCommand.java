// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.Shooter;

public class ShootCommand extends Command {
  Indexer s_Indexer;
  LedSubsystem s_Led;
  Shooter s_Shooter;
  boolean s_isDone;
  double RPM;
  double downRPM;
  /** Creates a new autoShoot. */
  public ShootCommand(Indexer s_Indexer,LedSubsystem s_Led,Shooter s_Shooter, double RPM,double downRPM) {
    this.s_Indexer = s_Indexer;
    this.s_Led = s_Led;
    this.s_Shooter = s_Shooter;
    this.RPM = RPM;
    this.downRPM = downRPM;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Indexer,s_Shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_isDone=false;
    s_Led.setShootMode(true);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   // s_Led.redPulse();
   
      s_Shooter.shooterSet(RPM,downRPM);
    
   if (s_Shooter.isShooterAtSetpoint()){
   // s_Led.blue();
   s_Indexer.manualIndex(0.75);
    Timer.delay(0.5);
    s_isDone = true;
   } }
    //s_Led.autoArm();
    
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Shooter.shooterIdle();
    s_Indexer.manualIndex(0.0);
    s_Led.setShootMode(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return s_isDone;
  }
}
