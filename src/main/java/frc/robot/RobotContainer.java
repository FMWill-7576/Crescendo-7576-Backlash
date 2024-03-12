// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.IntakeNote;
import frc.robot.commands.autoShoot;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.*;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import java.io.File;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // The robot's subsystems and commands are defined here...
  //private final Shooter s_Shooter = new Shooter();
  private final LedSubsystem s_Led = new LedSubsystem();
  private final Vision s_Vision = new Vision();
  private final Intake s_Intake = new Intake();
  public final SwerveSubsystem s_Swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),                                                                      "swerve/neo"));
  private final Arm s_Arm = new Arm();    
  private final Indexer s_Indexer = new Indexer() ;  
  private final Shooter s_Shooter = new Shooter();
  private final VictorClimber s_VictorClimber = new VictorClimber();                                                                                                                                     
  // CommandJoystick rotationController = new CommandJoystick(1);
  // Replace with CommandPS4Controller or CommandJoystick if needed
 // CommandJoystick driverController = new CommandJoystick(1);
  // CommandJoystick driverController   = new CommandJoystick(3);//(OperatorConstants.DRIVER_CONTROLLER_PORT);
  CommandPS5Controller driverPS5 = new CommandPS5Controller(1);
  CommandXboxController operatorXbox = new CommandXboxController(0);
  private SlewRateLimiter translationLimiter  = new SlewRateLimiter(3.2);
  private SlewRateLimiter strafeLimiter  = new SlewRateLimiter(3.2);
  private SlewRateLimiter rotationLimiter  = new SlewRateLimiter(3.2);
  private double speedRate = 1.0;
  
 



  SendableChooser<Command> m_chooser = new SendableChooser<>();
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    NamedCommands.registerCommand("runIntake",
     s_Intake.run(()-> s_Intake.manualIntake(0.85)).until(()->s_Indexer.isNoteInIndexer()));
     NamedCommands.registerCommand("shootWoofer",
     s_Shooter.run(()->
      s_Shooter.shooterSet(5500,5750)).
      until(()->s_Shooter.isShooterAtSetpoint()).
      andThen(s_Indexer.run(()-> s_Indexer.manualIndex(0.75))).
      until(()->!s_Indexer.isNoteInIndexer()));

      NamedCommands.registerCommand("autoAlign",
      new autoShoot(s_Indexer, s_Arm, s_Led, s_Swerve, s_Shooter));






    
    // Configure the trigger bindings
    configureBindings();

   /*  AbsoluteDriveAdv closedAbsoluteDriveAdv = 
    new AbsoluteDriveAdv(s_Swerve,
      () -> -MathUtil.applyDeadband(driverPS5.getLeftY(),
                                   OperatorConstants.LEFT_Y_DEADBAND),
      () -> -MathUtil.applyDeadband(driverPS5.getLeftX(),
                                   OperatorConstants.LEFT_X_DEADBAND),
      () -> -MathUtil.applyDeadband(driverPS5.getRightX(),
                                   OperatorConstants.RIGHT_X_DEADBAND),
      driverPS5.getHID()::getAButtonPressed,
      driverPS5.getHID()::getYButtonPressed,
      driverPS5.getHID()::getBButtonPressed,
      driverPS5.getHID()::getXButtonPressed); */

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    Command driveFieldOrientedDirectAngle = s_Swerve.driveCommand(
        () -> 0.3 * -MathUtil.applyDeadband(driverPS5.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> 0.3 * -MathUtil.applyDeadband(driverPS5.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -driverPS5.getRightX(),
        () -> -driverPS5.getRightY());

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAngularVelocity = s_Swerve.driveCommand(
        () -> speedRate * translationLimiter.calculate(-MathUtil.applyDeadband(driverPS5.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND)),
        () -> speedRate * strafeLimiter.calculate(-MathUtil.applyDeadband(driverPS5.getLeftX(), OperatorConstants.LEFT_X_DEADBAND)),
        () -> speedRate * -MathUtil.applyDeadband(driverPS5.getRightX(), OperatorConstants.RIGHT_X_DEADBAND));

    Command driveFieldOrientedDirectAngleSim = s_Swerve.simDriveCommand(
        () ->  -MathUtil.applyDeadband(driverPS5.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () ->  -MathUtil.applyDeadband(driverPS5.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () ->  -driverPS5.getRightX());

  

    s_Swerve.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedAngularVelocity : driveFieldOrientedDirectAngleSim);

    
    s_Intake.setDefaultCommand(
       s_Intake.run(()-> s_Intake.manualIntake( -MathUtil.applyDeadband(operatorXbox.getLeftY(),0.1))));


    s_Indexer.setDefaultCommand(
      s_Indexer.run(()-> s_Indexer.manualIndex( -MathUtil.applyDeadband(operatorXbox.getLeftY(),0.1))));
      
         s_VictorClimber.setDefaultCommand(
      s_VictorClimber.run(()-> s_VictorClimber.manualclimb(0.0)));

    s_Arm.setDefaultCommand(s_Arm.run(()-> s_Arm.armHold()));

    s_Shooter.setDefaultCommand(s_Shooter.run(() -> s_Shooter.shooterIdle()));

   // Load a Choreo trajectory as a PathPlannerPath
     PathPlannerPath exampleChoreoTraj = PathPlannerPath.fromChoreoTrajectory("deneme path");
    ChoreoTrajectory traj = Choreo.getTrajectory("deneme path"); //

    m_chooser.setDefaultOption("mid pre+1", s_Swerve.getAutonomousCommand("mid pre+1"));
    SmartDashboard.putData("OTONOM", m_chooser);
      m_chooser.addOption("deneme 6", s_Swerve.getAutonomousCommand("deneme auto 6"));
      m_chooser.addOption("mid pre +1", s_Swerve.getAutonomousCommand("mid pre+1"));
      m_chooser.addOption("bottom pre+1", s_Swerve.getAutonomousCommand("bottom pre+1"));
      m_chooser.addOption("mid 4 piece", s_Swerve.getAutonomousCommand("mid 4 piece"));
      m_chooser.addOption("mid pre", s_Swerve.getAutonomousCommand("mid pre"));
      m_chooser.addOption("mid pre+chaos", s_Swerve.getAutonomousCommand("mid pre+chaos"));
      m_chooser.addOption("mid pre+chaos down", s_Swerve.getAutonomousCommand("mid pre+chaos down"));
      m_chooser.addOption("bottom pre chaos", s_Swerve.getAutonomousCommand("bottom pre chaos"));
      m_chooser.addOption("top pre chaos", s_Swerve.getAutonomousCommand("top pre chaos"));
      m_chooser.addOption("top pre", s_Swerve.getAutonomousCommand("top pre"));
      m_chooser.addOption("top pre+1", s_Swerve.getAutonomousCommand("top pre+1"));
      m_chooser.addOption("bottom pre taxi", s_Swerve.getAutonomousCommand("bottom pre taxi"));
      m_chooser.addOption("bottom pre taxi+1", s_Swerve.getAutonomousCommand("bottom pre taxi+1"));
      m_chooser.addOption("bottom pre chaos2", s_Swerve.getAutonomousCommand("bottom pre chaos2"));
      
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    driverPS5.options().onTrue((new InstantCommand(s_Swerve::zeroGyro)));
    /* driverPS5.L3().whileTrue(
        Commands.deferredProxy(() -> s_Swerve.driveToPose(
                                   new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
                              )); */

  driverPS5.triangle().whileTrue(s_Swerve.run
  (() -> s_Swerve.aimAtTarget
  (() -> -driverPS5.getLeftX(), () -> -driverPS5.getLeftY())));  
  
  
  driverPS5.square().whileTrue(new RepeatCommand(new InstantCommand(s_Swerve::lock, s_Swerve)));
  driverPS5.L1().onTrue(Commands.runOnce(() -> speedRate = 0.25));
  driverPS5.L1().onFalse(Commands.runOnce(() -> speedRate = 1.0));

  driverPS5.R1().onTrue(Commands.runOnce(() -> speedRate = 0.50));
  driverPS5.R1().onFalse(Commands.runOnce(() -> speedRate = 1.0));

  //driverPS5.R3().onTrue(Commands.runOnce(() -> s_Vision.setLimelightLed()));


  driverPS5.L2().whileTrue(
    new RunCommand(()-> s_Arm.armHome(),s_Arm).onlyIf(() -> !s_Arm.isArmHome()).until(()->s_Arm.isArmHome()).andThen(new IntakeNote(s_Indexer,s_Intake,s_Led)));

  driverPS5.circle().whileTrue(
    new autoShoot(s_Indexer, s_Arm, s_Led, s_Swerve,s_Shooter)
  // onlyIf(() -> s_Indexer.isNoteInIndexer()).
   // until(() -> s_Shooter.isShooterAtSetpoint()).
   // andThen(() ->s_Indexer.manualIndex(0.65))
   );

  
  operatorXbox.x().whileTrue(s_Intake.run(() -> s_Intake.manualIntake(0.75)));

  operatorXbox.rightBumper().whileTrue(
    s_Shooter.run(() -> s_Shooter.shooterSet(7100,7700)).
    until(() -> s_Shooter.isShooterAtSetpoint()).
    andThen(s_Indexer.run(() ->s_Indexer.manualIndex(0.65))));

  operatorXbox.b().whileTrue(s_Intake.run(() -> s_Intake.manualIntake(-0.85)));

  operatorXbox.b().whileTrue(s_Indexer.run(() -> s_Indexer.manualIndex(-0.85)));
  operatorXbox.b().whileTrue(s_Shooter.run(() -> s_Shooter.shooterSet(-400,-400)));

  
  operatorXbox.rightTrigger().whileTrue(
    s_Shooter.run(() -> s_Shooter.shooterSet(3000,3300)).
    until(() -> s_Shooter.isShooterAtSetpoint()).
    andThen(s_Indexer.run(() ->s_Indexer.manualIndex(0.65))));

  operatorXbox.leftTrigger().whileTrue(
    s_Shooter.run(() -> s_Shooter.shooterSet(4200,4700)).
    until(() -> s_Shooter.isShooterAtSetpoint()).
   andThen(s_Indexer.run(() ->s_Indexer.manualIndex(0.65))));

  operatorXbox.leftBumper().whileTrue(
    s_Shooter.run(() -> s_Shooter.shooterSet(6000,6600)).
    until(() -> s_Shooter.isShooterAtSetpoint()).
    andThen(s_Indexer.run(() ->s_Indexer.manualIndex(0.65))));

  operatorXbox.y().whileTrue(s_Arm.run(() -> s_Arm.armDrive(0.3)));

  operatorXbox.rightStick().whileTrue(s_VictorClimber.run(() -> s_VictorClimber.manualclimb(1.0)));

  operatorXbox.leftStick().whileTrue(s_VictorClimber.run(() -> s_VictorClimber.manualclimb(-1.0)));
  //operatorXbox.b().whileTrue(s_Arm.run(() -> s_Arm.armDrive(-0.45)));
 // operatorXbox.a().whileTrue(s_Arm.run(() -> s_Arm.armDrive(-0.3)));
 operatorXbox.a().whileTrue(s_Arm.run(()-> s_Arm.armDrive(-0.3)));
 operatorXbox.povUp().onTrue(s_Arm.run(()-> s_Arm.armSet(Rotation2d.fromDegrees(51.8))));
 operatorXbox.povLeft().onTrue(s_Arm.run(()-> s_Arm.armSet(Rotation2d.fromDegrees(-16.4))));//-26.2, -20.6
 operatorXbox.povDown().onTrue(s_Arm.run(()-> s_Arm.armSet(Rotation2d.fromDegrees(-40.7))));
 operatorXbox.povRight().whileTrue(s_Arm.run(()-> s_Arm.armSet(Rotation2d.fromDegrees(-8))));
 //operatorXbox.povDown().whileTrue(s_Arm.run(()-> s_Arm.armHold()));
 operatorXbox.x().whileTrue(s_Indexer.run(() -> s_Indexer.manualIndex(0.65)));
 operatorXbox.start().whileTrue(
    new RunCommand(()-> s_Arm.armHome(),s_Arm).onlyIf(() -> !s_Arm.isArmHome()).until(()->s_Arm.isArmHome()).andThen(new IntakeNote(s_Indexer,s_Intake,s_Led)));
 

  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
   // return s_Swerve.getAutonomousCommand("New Auto");
   return m_chooser.getSelected();
  }

  public void setDriveMode()
  {
    //s_Swerve.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    s_Swerve.setMotorBrake(brake);
  }
}
