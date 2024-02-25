package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkMaxAlternateEncoder.Type;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import com.revrobotics.SparkPIDController;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;
//@SuppressWarnings("unused")

public class Arm extends SubsystemBase {
  private static double kS = 0.00;
  private static double kG = 0.4;
  private static double kV = 0.0;
  public double calculatedkG;
  private ArmFeedforward armFeedforward;
  public static double armSpeedRate =  1.0;
  private CANSparkMax armMotor;
  //private CANSparkMax armMotor2;
  private final SparkPIDController armController;
  //private final SparkPIDController armController2;
  private RelativeEncoder integratedArmEncoder;
 // private RelativeEncoder integratedArmEncoder2;
  private RelativeEncoder throughbEncoder;
  /* private final TrapezoidProfile.Constraints m_constraints;
  private  TrapezoidProfile.State  m_start;
  private TrapezoidProfile.State  m_end;
  private TrapezoidProfile profile;
  private TrapezoidProfile.State m_goal;
  private TrapezoidProfile.State m_setpoint;
  private static double kDt; */
  private ArmFeedforward feedforward;
  private static double setpoint;
  // Ks = 0.42 VOLT UNUTMA!!!!

  public Arm() {
      armMotor  = new CANSparkMax(11, MotorType.kBrushless);
      //armMotor2  = new CANSparkMax(65, MotorType.kBrushless); 
      armController = armMotor.getPIDController();
     // armController2 = armMotor2.getPIDController();
      feedforward = new ArmFeedforward(kS, kG, kV);
     integratedArmEncoder = armMotor.getEncoder();
     //integratedArmEncoder2 = armMotor2.getEncoder();
     throughbEncoder = armMotor.getAlternateEncoder(Type.kQuadrature,8192);
     armMotorConfig();

  }
 



      public void armMotorConfig() { 
        armMotor.restoreFactoryDefaults();
      //  armMotor2.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(armMotor, Usage.kPositionOnly,true);
       // CANSparkMaxUtil.setCANSparkMaxBusUsage(armMotor2, Usage.kPositionOnly,true);
        armMotor.setSmartCurrentLimit(40);
       // armMotor2.setSmartCurrentLimit(40);
        armMotor.setInverted(true);
        armMotor.setIdleMode(IdleMode.kBrake);
      //  armMotor2.setInverted(true);
      //  armMotor2.setIdleMode(IdleMode.kBrake); 
        integratedArmEncoder.setPositionConversionFactor((1/64) * (22/68)); 
        integratedArmEncoder.setPosition(-46.41);
       // integratedArmEncoder2.setPositionConversionFactor(7.742); 
       // integratedArmEncoder.setVelocityConversionFactor(7.742); 
       // integratedArmEncoder2.setPosition(-64.5); 
       // integratedArmEncoder2.setVelocityConversionFactor(7.742);
        throughbEncoder.setPositionConversionFactor(180/(68/22));
        throughbEncoder.setPosition(-46.41);   // -47.41
        throughbEncoder.setInverted(true);
        armController.setP(Constants.ArmConstants.armKP);
        armController.setI(Constants.ArmConstants.armKI);
        armController.setD(Constants.ArmConstants.armKD);
        armController.setIMaxAccum(Constants.ArmConstants.armKIMax,0);
        armController.setIZone(Constants.ArmConstants.armKIZone);
        armController.setFeedbackDevice(throughbEncoder);
      //  armController2.setP(Constants.ArmConstants.armKP);
      //  armController2.setI(Constants.ArmConstants.armKI);
      //  armController2.setD(Constants.ArmConstants.armKD);
      //  armController2.setIMaxAccum(Constants.ArmConstants.armKIMax,0);
      //  armController2.setIZone(Constants.ArmConstants.armKIZone);
     //   armController2.setOutputRange(-0.48, 0.48);
      //  armController2.setFeedbackDevice(integratedArmEncoder2);
        armMotor.enableVoltageCompensation(12.0);
        armMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        armMotor.setSoftLimit(SoftLimitDirection.kForward, 65);
        armMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        armMotor.setSoftLimit(SoftLimitDirection.kReverse, -47);

        armMotor.setClosedLoopRampRate(0.15);
        armMotor.setOpenLoopRampRate(0.1);
  
      //  armMotor2.enableVoltageCompensation(12.0);
      //  armMotor2.follow(armMotor);
     
        armController.setOutputRange(-1.0, 1.0);
        //integratedArmEncoder.setReverseDirection(false); // bu
        Timer.delay(0.2);
        armMotor.burnFlash();
      //  armMotor2.burnFlash();
        Timer.delay(0.2);
    }

      public void armSet(Rotation2d angle) {

        armController.setReference(
         angle.getDegrees(),
         CANSparkMax.ControlType.kPosition,
         0,
         feedforward.calculate(angle.getRadians(),0.00),
         ArbFFUnits.kVoltage);
         
       setpoint=angle.getDegrees();
      }

      public void armHold(){
        armSet(Rotation2d.fromDegrees(throughbEncoder.getPosition()));
      }
     

     public void armUp(){
     // armSet(Rotation2d.fromDegrees(150.0));
      armDrive(0.31);
     }

     public void armScore(){
       armSet(Rotation2d.fromDegrees
       (36.2));
      }

     public void armGrip(){
      armSet(Rotation2d.fromDegrees
      (239.5));
    }
    public void armPickUp(){
      armSet(Rotation2d.fromDegrees
      (14.0));
    }

    public void armScoreReverse(){
      armSet(Rotation2d.fromDegrees
      (140.0));

    }

     public void armDrive(double armPercentage){
      armMotor.set(armPercentage);
      //armMotor2.set(armPercentage);
    }    
    
    public void armVoltage(double voltage){
      armMotor.setVoltage(voltage);
    }

     public void armHome(){
      armSet(Rotation2d.fromDegrees(-48.0));
     }

     public void armDown(){
      //armSet(Rotation2d.fromDegrees(200.0));
      armDrive(-0.31);
     }

     public void armStop(){
      armDrive(0.0);
     }


     public void armReset(){
      throughbEncoder.setPosition(-68.0);
      integratedArmEncoder.setPosition(-68.0);
      //integratedArmEncoder2.setPosition(-68.0);
     }

    @Override
    public void periodic() {
  
      // This method will be called once per scheduler run
      SmartDashboard.putNumber("arm encoder" , integratedArmEncoder.getPosition());
      SmartDashboard.putNumber("bore encoder" , throughbEncoder.getPosition());
      SmartDashboard.putNumber("setpoint",setpoint);
      //SmartDashboard.putNumber("arm speed",integratedArmEncoder.getVelocity());
      SmartDashboard.putNumber("arm output", armMotor.getAppliedOutput());
       SmartDashboard.putNumber("arm output amps", armMotor.getOutputCurrent());

    }
}