package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
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
  private static boolean isArmHome;
  public double calculatedkG;
  private ArmFeedforward armFeedforward;
  public static double armSpeedRate =  1.0;
  private CANSparkMax armMotor;
  //private CANSparkMax armMotor2;
  private final SparkPIDController armController;
  private final PowerDistribution examplePD;
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
  private DigitalInput limitSwitch;
  // Ks = 0.42 VOLT UNUTMA!!!!

  public Arm() {
      armMotor  = new CANSparkMax(11, MotorType.kBrushless);
      limitSwitch = new DigitalInput(9);
       examplePD = new PowerDistribution(0, ModuleType.kCTRE);
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
        CANSparkMaxUtil.setCANSparkMaxBusUsage(armMotor, Usage.kPositionOnly,true);
        armMotor.setSmartCurrentLimit(45);
        armMotor.setInverted(true);
        armMotor.setIdleMode(IdleMode.kBrake);
       // integratedArmEncoder.setPositionConversionFactor((1/64) * (22/68)); 
        integratedArmEncoder.setPosition(-42.612);
        throughbEncoder.setPositionConversionFactor(180/(68/22));
        throughbEncoder.setPosition(-42.612);   // -46.41
        throughbEncoder.setInverted(true);
        armController.setP(Constants.ArmConstants.armKP);
        armController.setI(Constants.ArmConstants.armKI);
        armController.setD(Constants.ArmConstants.armKD);
        armController.setIMaxAccum(Constants.ArmConstants.armKIMax,0);
        armController.setIZone(Constants.ArmConstants.armKIZone);
        armController.setFeedbackDevice(throughbEncoder);
        armMotor.enableVoltageCompensation(12.0);
        armMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        armMotor.setSoftLimit(SoftLimitDirection.kForward, 60);
        armMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        armMotor.setSoftLimit(SoftLimitDirection.kReverse, -42.8f);

        armMotor.setClosedLoopRampRate(0.12);
        armMotor.setOpenLoopRampRate(0.12);
     
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
     
     public boolean isArmHome(){
      //return !limitSwitch.get();
      return Math.abs(-42.612 - throughbEncoder.getPosition())<0.45;
     }
     public void armUp(){
     // armSet(Rotation2d.fromDegrees(150.0));
      armDrive(0.31);
     }


    public boolean isArmAtSetpoint(){
      return getArmError() < 0.45;
    }

     public void armDrive(double armPercentage){
      armMotor.set(armPercentage);
    
      //armMotor2.set(armPercentage);
    }    
    
    public void armVoltage(double voltage){
      armMotor.setVoltage(voltage);
    }

     public void armHome(){
      armSet(Rotation2d.fromDegrees(-42.7));
      
     }

     public void armDown(){
      //armSet(Rotation2d.fromDegrees(200.0));
      armDrive(-0.31);
     }

     public void armStop(){
      armDrive(0.0);
     }

     public double getArmError() {
      return Math.abs(setpoint - throughbEncoder.getPosition());
  }


     public void armReset(){
      throughbEncoder.setPosition(-68.0);
      integratedArmEncoder.setPosition(-68.0);
      //integratedArmEncoder2.setPosition(-68.0);
     }

    @Override
    public void periodic() {
      
      if (-44 < throughbEncoder.getPosition() && throughbEncoder.getPosition() < -42.4){
        isArmHome = true;
      }
      else{
        isArmHome = false;
      }
  
      // This method will be called once per scheduler run
      SmartDashboard.putNumber("integrated arm encoder" , integratedArmEncoder.getPosition());
      SmartDashboard.putNumber("arm encoder" , throughbEncoder.getPosition());
      SmartDashboard.putNumber("setpoint",setpoint);
      //SmartDashboard.putNumber("arm speed",integratedArmEncoder.getVelocity());
      SmartDashboard.putNumber("arm output duty", armMotor.getAppliedOutput());
       SmartDashboard.putNumber("arm output amps", armMotor.getOutputCurrent());
       SmartDashboard.putNumber("arm output voltage", armMotor.getBusVoltage());
     //  SmartDashboard.putNumber("arm nominal voltage", armMotor.getVoltageCompensationNominalVoltage());
       SmartDashboard.putNumber("pdp voltage", examplePD.getVoltage());
       SmartDashboard.putBoolean("limit switch", isArmHome());

    }
}