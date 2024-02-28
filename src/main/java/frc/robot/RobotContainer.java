// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.*;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import java.io.File;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // The robot's subsystems and commands are defined here...
  //private final Shooter s_Shooter = new Shooter();
  //private final LedSubsystem LedSubsystem = new LedSubsystem();
 // private final Vision s_Vision = new Vision();
  private final Intake s_Intake = new Intake();
  private final SwerveSubsystem s_Swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),                                                                      "swerve/neo"));
  private final Arm s_Arm = new Arm();    
  private final Indexer s_Indexer = new Indexer() ;  
  private final Shooter s_Shooter = new Shooter();                                                                                                                                    
  // CommandJoystick rotationController = new CommandJoystick(1);
  // Replace with CommandPS4Controller or CommandJoystick if needed
 // CommandJoystick driverController = new CommandJoystick(1);
  // CommandJoystick driverController   = new CommandJoystick(3);//(OperatorConstants.DRIVER_CONTROLLER_PORT);
  CommandXboxController driverXbox = new CommandXboxController(2);
  CommandXboxController operatorXbox = new CommandXboxController(0);


  SendableChooser<Command> m_chooser = new SendableChooser<>();
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();

    AbsoluteDriveAdv closedAbsoluteDriveAdv = 
    new AbsoluteDriveAdv(s_Swerve,
      () -> -MathUtil.applyDeadband(driverXbox.getLeftY(),
                                   OperatorConstants.LEFT_Y_DEADBAND),
      () -> -MathUtil.applyDeadband(driverXbox.getLeftX(),
                                   OperatorConstants.LEFT_X_DEADBAND),
      () -> -MathUtil.applyDeadband(driverXbox.getRightX(),
                                   OperatorConstants.RIGHT_X_DEADBAND),
      driverXbox.getHID()::getAButtonPressed,
      driverXbox.getHID()::getYButtonPressed,
      driverXbox.getHID()::getBButtonPressed,
      driverXbox.getHID()::getXButtonPressed);

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    Command driveFieldOrientedDirectAngle = s_Swerve.driveCommand(
        () -> 0.3 * -MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> 0.3 * -MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -driverXbox.getRightX(),
        () -> -driverXbox.getRightY());

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAngularVelocity = s_Swerve.driveCommand(
        () -> 0.4 * -MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> 0.4 * -MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> 0.8 * -MathUtil.applyDeadband(driverXbox.getRightX(), OperatorConstants.RIGHT_X_DEADBAND));

    Command driveFieldOrientedDirectAngleSim = s_Swerve.simDriveCommand(
        () ->  -MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () ->  -MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () ->  -driverXbox.getRightX());

  

    s_Swerve.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedAngularVelocity : driveFieldOrientedDirectAngleSim);

    
    s_Intake.setDefaultCommand(
       s_Intake.run(()-> s_Intake.manualIntake(0.0)));


    s_Indexer.setDefaultCommand(
      s_Indexer.run(()-> s_Indexer.manualIndex(0.0)));

    s_Arm.setDefaultCommand(s_Arm.run(()-> s_Arm.armHold()));

    s_Shooter.setDefaultCommand(s_Shooter.run(() -> s_Shooter.shootDriveManual(operatorXbox.getLeftTriggerAxis())));

    m_chooser.setDefaultOption("exampleAuto", s_Swerve.getAutonomousCommand("New Auto"));
  //  m_chooser.addOption("auto2", s_Swerve.getAutonomousCommand("null"));
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

    driverXbox.leftBumper().onTrue((new InstantCommand(s_Swerve::zeroGyro)));
   // new JoystickButton(driverXbox, 3).onTrue(new InstantCommand(s_Swerve::addFakeVisionReading));
    driverXbox.leftStick().whileTrue(
        Commands.deferredProxy(() -> s_Swerve.driveToPose(
                                   new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
                              ));
  driverXbox.x().whileTrue(new RepeatCommand(new InstantCommand(s_Swerve::lock, s_Swerve)));

  
  operatorXbox.x().whileTrue(s_Intake.run(() -> s_Intake.intake()));
  //operatorXbox.b().toggleOnTrue(s_Intake.run(() -> s_Intake.stop()));
  operatorXbox.y().whileTrue(s_Arm.run(() -> s_Arm.armDrive(0.7)));
  //operatorXbox.b().whileTrue(s_Arm.run(() -> s_Arm.armDrive(-0.45)));
 // operatorXbox.a().whileTrue(s_Arm.run(() -> s_Arm.armDrive(-0.3)));
 operatorXbox.a().whileTrue(s_Arm.run(()-> s_Arm.armDrive(-0.7)));
 operatorXbox.povUp().onTrue(s_Arm.run(()-> s_Arm.armSet(Rotation2d.fromDegrees(55.0))));
 operatorXbox.povLeft().onTrue(s_Arm.run(()-> s_Arm.armSet(Rotation2d.fromDegrees(-20.0))));
 operatorXbox.povDown().onTrue(s_Arm.run(()-> s_Arm.armSet(Rotation2d.fromDegrees(-46.0))));
 operatorXbox.povRight().whileTrue(s_Arm.run(()-> s_Arm.armSet(Rotation2d.fromDegrees(20.0))));
 //operatorXbox.povDown().whileTrue(s_Arm.run(()-> s_Arm.armHold()));
 operatorXbox.x().whileTrue(s_Indexer.run(() -> s_Indexer.manualIndex(1.0)));

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
