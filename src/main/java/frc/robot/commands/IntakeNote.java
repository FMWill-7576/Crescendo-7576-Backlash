// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LedSubsystem;

public class IntakeNote extends Command {
  Intake s_Intake;
  Indexer s_Indexer;
  LedSubsystem s_Led;
  boolean s_isDone;
  /** Creates a new IntakeNote. */
  public IntakeNote(Indexer s_Indexer, Intake s_Intake, LedSubsystem s_Led) {
    this.s_Indexer = s_Indexer;
    this.s_Intake = s_Intake;
    this.s_Led = s_Led;
    addRequirements(s_Indexer,s_Led,s_Intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_isDone = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!s_Indexer.isNoteInIndexer()){
      s_Intake.manualIntake(0.85);
      s_Indexer.manualIndex(0.5);
      // s_Led.noNote();
    } else {
      Timer.delay(0.07576);
      s_Intake.manualIntake(0.0);
      s_Indexer.manualIndex(0.0);
      //s_Led.yesNote();
      s_isDone = true;
    }

 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Intake.manualIntake(0.0);
    s_Indexer.manualIndex(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return s_isDone;
  }
}
