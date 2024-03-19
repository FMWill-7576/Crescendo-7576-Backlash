// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
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

  public static final double ROBOT_MASS = 57.0; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag

  public static final class AutonConstants
  {

    public static final PIDConstants TRANSLATION_PID = new PIDConstants(4.0, 0, 0);
    public static final PIDConstants ANGLE_PID   = new PIDConstants(4.8, 0, 0.003);
  }
 public static final class ArmConstants {
  

 //   public static final double armConversionPositionFactor = 360.0/71.0/7.0/2.4923 ;

    public static final double armKP = 0.0790 ;
    public static final double armKI = 0.00000 ;
    public static final double armKIMax = 0.00680;
    public static final double armKIZone = 2.5;
    public static final double armKD = 0.00200;//300
    public static final double armKFF = 0.0 ;

    public static final  IdleMode armNeutralMode = IdleMode.kBrake ;
    public static final boolean armInvert = false ;

    public static final double voltageComp = 12.0 ;
    public static final Boolean voltageCompBoolean = true ;
    public static final int armContinuousCurrentLimit = 40;

  //  public static final double stickDeadband = 0.04;

  }
  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND  = 0.08;
    public static final double LEFT_Y_DEADBAND  = 0.08;
    public static final double RIGHT_X_DEADBAND = 0.08;
    public static final double TURN_CONSTANT    = 7;
  }

 public static class Vision {
        public static final String kCameraName = "Limelight";
        //public static final String kCameraName = "USB_Camera";
        // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
        public static final Transform3d kRobotToCam =
                new Transform3d(new Translation3d( -0.25665,-0.8644, 0.578), new Rotation3d(0, Math.toRadians(31.0),0));

        //public static final Transform3d kRobotToCam =
                //new Transform3d(new Translation3d(0.264922,0.2465578, 0.2182876), new Rotation3d(0, Math.toRadians(22.09),Math.toRadians(5)));

        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout kTagLayout =
                AprilTagFields.kDefaultField.loadAprilTagLayoutField();

        // The standard deviations of our vision estimated poses, which affect correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    }
}
