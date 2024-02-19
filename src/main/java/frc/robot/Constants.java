// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag

  public static final class AutonConstants
  {

    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
    public static final PIDConstants ANGLE_PID   = new PIDConstants(0.4, 0, 0.01);
  }
 public static final class ArmConstants {
    public static final int armMotorID = 9 ;



    public static final double armConversionPositionFactor = 360.0/71.0/7.0/2.4923 ;

    public static final double armKP = 0.00715 ;
    public static final double armKI = 0.00000 ;
    public static final double armKIMax = 0.00680;
    public static final double armKIZone = 2.5;
    public static final double armKD = 0.00500;//900
    public static final double armKFF = 0.0 ;

    public static final  IdleMode armNeutralMode = IdleMode.kBrake ;
    public static final boolean armInvert = false ;

    public static final double voltageComp = 12.0 ;
    public static final Boolean voltageCompBoolean = true ;
    public static final int armContinuousCurrentLimit = 40;

    public static final double stickDeadband = 0.04;

  }
  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND  = 0.1;
    public static final double LEFT_Y_DEADBAND  = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }

public static class VisionConstants {

    public static final Transform3d robotToCam = null;
    public static String cameraName;

}
}