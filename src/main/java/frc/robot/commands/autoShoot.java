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
  public autoShoot(Indexer s_Indexer, Arm s_Arm, LedSubsystem s_Led, SwerveSubsystem s_Swerve) {
    this.s_Indexer = s_Indexer;
    this.s_Arm = s_Arm;
    this.s_Led = s_Led;
    this.s_Swerve = s_Swerve;
    tableArm = new InterpolatingDoubleTreeMap();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Indexer,s_Led,s_Shooter,s_Arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
       
    tableArm.put(125.0, 450.0);
    tableArm.put(200.0, 510.0);
    tableArm.put(268.0, 525.0);
    tableArm.put(312.0, 550.0);
    tableArm.put(326.0, 650.0);


    tableShooter.put(125.0, 450.0);
    tableShooter.put(200.0, 510.0);
    tableShooter.put(268.0, 525.0);
    tableShooter.put(312.0, 550.0);
    tableShooter.put(326.0, 650.0);

  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Arm.armSet(Rotation2d.fromDegrees(tableArm.get(s_Swerve.getVisionDistance())));
    s_Shooter.shooterSet(tableShooter.get(s_Swerve.getVisionDistance()));
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
