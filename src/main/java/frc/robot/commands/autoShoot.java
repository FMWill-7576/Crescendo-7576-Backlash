// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class autoShoot extends Command {
  Indexer s_Indexer;
  SwerveSubsystem s_Swerve;
  Arm s_Arm;
  LedSubsystem s_Led;
  Shooter s_Shooter;
  boolean s_isDone;
   InterpolatingDoubleTreeMap tableArm;
   InterpolatingDoubleTreeMap tableShooter;
  /** Creates a new autoShoot. */
  public autoShoot(Indexer s_Indexer, Arm s_Arm, LedSubsystem s_Led, SwerveSubsystem s_Swerve, Shooter s_Shooter) {
    this.s_Indexer = s_Indexer;
    this.s_Arm = s_Arm;
    this.s_Led = s_Led;
    this.s_Swerve = s_Swerve;
    this.s_Shooter = s_Shooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Indexer,s_Led,s_Shooter,s_Arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    tableArm = new InterpolatingDoubleTreeMap();
    tableShooter = new InterpolatingDoubleTreeMap();
       

    tableArm.put(1.49, -39.3);
    tableArm.put(1.74, -28.1);
    tableArm.put(1.98, -22.8);
    tableArm.put(2.05, -22.5);
    tableArm.put(2.3, -19.0);
    tableArm.put(2.69, -16.1);
    tableArm.put(2.96, -15.7);
    tableArm.put(3.16, -14.0);
    tableArm.put(3.55, -11.0);
    tableArm.put(3.86, -8.4);
     tableArm.put(4.08, -7.7);
     tableArm.put(4.67, -9.4);


   // tableShooter.put(125.0, 450.0);
    //tableShooter.put(200.0, 510.0);
    tableShooter.put(2.96, 6600.0);
    tableShooter.put(1.74, 4000.0);
    tableShooter.put(2.05, 4200.0);
    tableShooter.put(4.08, 7200.0);
    tableShooter.put(4.55, 7300.0);
    tableShooter.put(5.0, 7400.0);
    s_isDone=false;

  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Arm.armSet(Rotation2d.fromDegrees(tableArm.get(s_Swerve.getVisionDistance())));
   s_Shooter.shooterSet(tableShooter.get(s_Swerve.getVisionDistance()),tableShooter.get(s_Swerve.getVisionDistance())+600);
    if (s_Shooter.isShooterAtSetpoint()){
      s_Shooter.shooterSet(tableShooter.get(s_Swerve.getVisionDistance()),tableShooter.get(s_Swerve.getVisionDistance())+600);
      s_Indexer.manualIndex(0.65);
      if(!s_Indexer.isNoteInIndexer())
    s_isDone = true;
   }
    //s_Led.autoArm();
    
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return s_isDone;
  }
}
