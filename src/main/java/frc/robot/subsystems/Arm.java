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
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkMaxAlternateEncoder.Type;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import com.revrobotics.SparkPIDController;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;
//@SuppressWarnings("unused")

public class Arm extends SubsystemBase {
  private static double kS = 0.3;
  private static double kG = 0.45;
  private static double kV = 0.13;
  public double calculatedkG;
  private ArmFeedforward armFeedforward;
  public static double armSpeedRate =  1.0;
  private CANSparkMax armMotor;
  private CANSparkMax armMotor2;
  private final SparkPIDController armController;
  private final SparkPIDController armController2;
  private RelativeEncoder integratedArmEncoder;
  private RelativeEncoder integratedArmEncoder2;
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

  public Arm() {
      armMotor  = new CANSparkMax(11, MotorType.kBrushless);
      armMotor2  = new CANSparkMax(65, MotorType.kBrushless); 
      armController = armMotor.getPIDController();
      armController2 = armMotor2.getPIDController();
      armFeedforward = new ArmFeedforward(kS, kG, kV);
     /* m_start = new TrapezoidProfile.State(-68, 0);
     m_end = new TrapezoidProfile.State(30, 0);
     profile = new TrapezoidProfile(m_constraints,m_end,m_start);
      m_goal = new TrapezoidProfile.State();
      m_constraints = new TrapezoidProfile.Constraints(2, 0.3);
      m_setpoint = new TrapezoidProfile.State();
      kDt = 0.02; */
      feedforward = new ArmFeedforward(kS, kG, kV);
    
    
     integratedArmEncoder = armMotor.getEncoder();
     integratedArmEncoder2 = armMotor2.getEncoder();
     throughbEncoder = armMotor.getAlternateEncoder(Type.kQuadrature,8192);
     armMotorConfig();

  }
 



      public void armMotorConfig() { 
        armMotor.restoreFactoryDefaults();
        armMotor2.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(armMotor, Usage.kPositionOnly,true);
        CANSparkMaxUtil.setCANSparkMaxBusUsage(armMotor2, Usage.kPositionOnly,true);
        armMotor.setSmartCurrentLimit(40);
        armMotor2.setSmartCurrentLimit(40);
        armMotor.setInverted(false);
        armMotor.setIdleMode(IdleMode.kBrake);
        armMotor2.setInverted(true);
        armMotor2.setIdleMode(IdleMode.kBrake); 
        integratedArmEncoder.setPositionConversionFactor(7.742); 
        integratedArmEncoder.setPosition(-64.5);
        integratedArmEncoder2.setPositionConversionFactor(7.742); 
        integratedArmEncoder.setVelocityConversionFactor(7.742); 
        integratedArmEncoder2.setPosition(-64.5); 
        integratedArmEncoder2.setVelocityConversionFactor(7.742);
        throughbEncoder.setPositionConversionFactor(193.5);
        throughbEncoder.setPosition(-64.5);
        throughbEncoder.setInverted(true);
        armController.setP(Constants.ArmConstants.armKP);
        armController.setI(Constants.ArmConstants.armKI);
        armController.setD(Constants.ArmConstants.armKD);
        armController.setIMaxAccum(Constants.ArmConstants.armKIMax,0);
        armController.setIZone(Constants.ArmConstants.armKIZone);
        armController.setFeedbackDevice(throughbEncoder);
        armController2.setP(Constants.ArmConstants.armKP);
        armController2.setI(Constants.ArmConstants.armKI);
        armController2.setD(Constants.ArmConstants.armKD);
        armController2.setIMaxAccum(Constants.ArmConstants.armKIMax,0);
        armController2.setIZone(Constants.ArmConstants.armKIZone);
        armController2.setOutputRange(-0.48, 0.48);
        armController2.setFeedbackDevice(integratedArmEncoder2);
        armMotor.enableVoltageCompensation(12.0);
        armMotor2.enableVoltageCompensation(12.0);
        armMotor2.follow(armMotor);
        armController.setOutputRange(-0.48, 0.48);
        //integratedArmEncoder.setReverseDirection(false); // bu
        Timer.delay(0.1);
        armMotor.burnFlash();
        armMotor2.burnFlash();
        Timer.delay(0.1);
    }

      public void armSet(Rotation2d angle) {
        armController.setReference(
         angle.getDegrees(),
         CANSparkMax.ControlType.kPosition,
         0,
         feedforward.calculate(angle.getRadians(),0),
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
      integratedArmEncoder2.setPosition(-68.0);
     }

    @Override
    public void periodic() {
      //if (integratedArmEncoder.getPosition() > 85.0 && integratedArmEncoder.getPosition() < 95.0 )
     // {calculatedkG = 0.0; }
     // else if (integratedArmEncoder.getPosition()> 95.0){
      //  calculatedkG = -0.07;
     // }
     // else {
     // calculatedkG = 0.06 ; }
     //  m_setpoint = profile.calculate(kDt);
     // var profile = new TrapezoidProfile(m_constraints, m_goal, m_setpoint);

      // This method will be called once per scheduler run
      SmartDashboard.putNumber("arm encoder" , integratedArmEncoder.getPosition());
      SmartDashboard.putNumber("arm encoder2" , integratedArmEncoder2.getPosition());
      SmartDashboard.putNumber("bore encoder" , throughbEncoder.getPosition());
      SmartDashboard.putNumber("setpoint",setpoint);
      //SmartDashboard.putNumber("arm speed",integratedArmEncoder.getVelocity());
      SmartDashboard.putNumber("arm output", armMotor.getAppliedOutput());
      SmartDashboard.putNumber("arm output 2", armMotor2.getAppliedOutput());
      


      //SmartDashboard.putNumber("arm distance" , integratedArmEncoder.getDistance());
    }
}