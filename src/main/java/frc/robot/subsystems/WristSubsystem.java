package frc.robot.subsystems;

import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import java.util.Map;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;

public class WristSubsystem extends SubsystemBase{
    private  SparkMax m_Wrist; 
    private  SparkMaxConfig c_Wrist = new SparkMaxConfig();
    AbsoluteEncoder absEncoder;
    private AbsoluteEncoderConfig c_EncoderConfig = new AbsoluteEncoderConfig();
    private RelativeEncoder relEncoder;
    private final SparkClosedLoopController wristController;

    private double pivot_zero_offset = .1078;  //wrist was zeroed vertically up 
                                            //agains the elevator for initial setpoints then was take above the top of the elevator and 
                                            //moved past the initial zero by this amount then rezeroed.
    private double wristSetpoint = 0.89; //-pivot_zero_offset
    private double allowableError = 0.1;

    private ShuffleboardTab tab = Shuffleboard.getTab("Tuning");  //angles used for shuffleboard; taken from 2024 fulcrum code
    private GenericEntry sbAngle = tab.add("Wrist Angle", 1)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0, "max", 90))
            .getEntry();

    public WristSubsystem() {      
        m_Wrist = new SparkMax(WristConstants.kWrist, MotorType.kBrushless);
        wristController = m_Wrist.getClosedLoopController();
        absEncoder = m_Wrist.getAbsoluteEncoder();
        relEncoder = m_Wrist.getEncoder();




        c_Wrist
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(40)
                .voltageCompensation(12)
                .inverted(false);
        c_Wrist.softLimit
            .forwardSoftLimit(getRelativeTarget(0.9-pivot_zero_offset))
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimit(getRelativeTarget(0.56))
            .reverseSoftLimitEnabled(true);
                
                
        c_Wrist.encoder
                
                .positionConversionFactor(1.0) // meters
                .velocityConversionFactor(1.0);//meters/sec
        //c_EncoderConfig.zeroOffset(0.25);
                
                
//                  .inverted(false);
        c_Wrist.closedLoop
                  .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                  .p(0.2, ClosedLoopSlot.kSlot0)//need to tune
                  .p(0.2, ClosedLoopSlot.kSlot1)
                  .outputRange(-1, 1, ClosedLoopSlot.kSlot0)
                  .outputRange(-1, 1, ClosedLoopSlot.kSlot1)

                  .maxMotion
                    .maxVelocity(2000, ClosedLoopSlot.kSlot0)
                    .maxAcceleration(10000, ClosedLoopSlot.kSlot0)
                    .maxVelocity(200, ClosedLoopSlot.kSlot1)
                    .maxAcceleration(1000, ClosedLoopSlot.kSlot1)
                    .allowedClosedLoopError(.1, ClosedLoopSlot.kSlot0)
                    .allowedClosedLoopError(.1, ClosedLoopSlot.kSlot1)
                    ;
        setWristRelativeEncoderFromAbsolute();
        
        m_Wrist.configure(c_Wrist, ResetMode.kResetSafeParameters,
                    PersistMode.kPersistParameters);
        
        
    }
    @Override
    public void periodic() {
      SmartDashboard.putNumber("Wrist Velocity", relEncoder.getVelocity());
      SmartDashboard.putNumber("Wrist Output", m_Wrist.getAppliedOutput());
      SmartDashboard.putNumber("Wrist Setpoint", wristSetpoint);
      SmartDashboard.putNumber("Wrist Pos Abs", absEncoder.getPosition());
      SmartDashboard.putNumber("Wrist Pos Rel", relEncoder.getPosition());
      SmartDashboard.putNumber("Wrist FF", getPositionFeedForward());
    
    }

     public void stop() {
        m_Wrist.set(0.0);
    }

     public double getPosition() {
        return (absEncoder.getPosition()-pivot_zero_offset)*2*3.14159*34.0/36.0+.35319;
    }

    public double getPositionFeedForward(){
        double kF = 0.5;
        return kF*-1*Math.sin(getPosition());
    }

    public void setWristRelativeEncoderFromAbsolute(){
      double absolutePosition = absEncoder.getPosition();
      double wristRatio = 36.0/34.0*25.0;
      relEncoder.setPosition(absolutePosition*wristRatio);


    }



    public double getSbAngle() {  //reference lines 26-30
        return sbAngle.getDouble(1.0);
    }

     public void manualWristMove(double move) {
        m_Wrist.set(move);
    }

    public void setWristSetpoint(double setpoint){
        wristSetpoint = setpoint;
    }

    public void maxMotionPosition(){
    wristController.setReference(wristSetpoint, ControlType.kMAXMotionPositionControl);

    }

    public double getWristSetPoint(){
        return wristSetpoint;
    }


    public void closedLoopWrist() {
        // m_FulcrumRight.set(m_Controller.calculate(e_FulcrumEncoder.getPosition(),
        // setPoint));
        
        double ff = getPositionFeedForward();
        double sp = getWristSetPoint();
        wristController.setReference(sp, ControlType.kPosition, ClosedLoopSlot.kSlot0, ff);
    }

    public void moveToSetpoint() {
      double ff = getPositionFeedForward();
      wristController.setReference(getRelativeTarget(wristSetpoint), ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, ff);
    }
  
    public void moveToSetpointSlower() {
      double ff = getPositionFeedForward();
      wristController.setReference(getRelativeTarget(wristSetpoint), ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot1, ff);
    }


    public double getRelativeTarget(double absSoluteTarget){
      double wristRatio = 36.0/34.0*25.0;
      return absSoluteTarget*wristRatio;
    }


    public boolean isAtAngle() {
        double error = getPosition() - wristSetpoint;
        return (Math.abs(error) < allowableError);
      }


    public void setWristStow(){
        setWristSetpoint(0.88-pivot_zero_offset);
      }
    
      public void setWristCoralIntake(){
        setWristSetpoint(0.70);
      }
    
      public void setWristCoralL1Dump(){
        setWristSetpoint(0.61);
      }
    
      public void setWristCoralL1(){
        setWristSetpoint(0.69-pivot_zero_offset);
      }
    
      public void setWristCoralL2(){
        setWristSetpoint(0.69-pivot_zero_offset); // other option .72
      }
    
      public void setWristCoralL3(){
        setWristSetpoint(0.69-pivot_zero_offset);
      }
    
      public void setWristCoralL4(){
        setWristSetpoint(0.62);
      }
    
      public void setWristAlgaeL2(){
        setWristSetpoint(0.74-pivot_zero_offset);
      }
    
      public void setWristAlgaeL3(){
        setWristSetpoint(0.79-pivot_zero_offset);
      }

      public void setWristAlgaeStow(){
        setWristSetpoint(0.74-pivot_zero_offset);
      }


 }

