package frc.robot.subsystems;


import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;



public class ClimbSubsystem extends SubsystemBase{

     // Initialize elevator SPARK. We will use MAXMotion position control for the elevator, so we also
  // need to initialize the closed loop controller and encoder.



    private  SparkMax m_Climb; 
    private  SparkMaxConfig c_Climb = new SparkMaxConfig();
    private SparkClosedLoopController climbClosedLoopController;
    private RelativeEncoder c_Encoder;   
    private double climbSetPoint = 0;
    private double allowableError = 20;
    

    

    public ClimbSubsystem() {      
        m_Climb = new SparkMax(ClimbConstants.kClimb, MotorType.kBrushless);
        c_Encoder = m_Climb.getEncoder();
        climbClosedLoopController =  m_Climb.getClosedLoopController();



        c_Climb
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(30)
                
                .inverted(false);
        c_Climb.softLimit
            .forwardSoftLimit(30.0)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimit(-300)//230
            .reverseSoftLimitEnabled(true);
                
                
        c_Climb.encoder
                
                .positionConversionFactor(1.0) // meters
                .velocityConversionFactor(1.0);//meters/sec

        c_Climb.closedLoop
                  .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                  .p(3.0)//need to tune
                  .i(0)
                  .d(0.0)
                  .velocityFF(0.0)
                  .maxOutput(1.00)
                  .minOutput(-1.0)
                  .maxMotion
                    .maxAcceleration(1.0)
                    .maxVelocity(1.0)
                    .allowedClosedLoopError(.1) //degrees
                    ;
        
        m_Climb.configure(c_Climb, ResetMode.kResetSafeParameters,
                    PersistMode.kPersistParameters);
        c_Encoder.setPosition(0.0);
        
    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climber Position", c_Encoder.getPosition());
    
    }

     public void stop() {
        m_Climb.set(0.0);
    }

     public double getPosition() {
        return c_Encoder.getPosition();
    }

   

     public void manualClimbMove(double move) {
        m_Climb.set(move);
    }

    public void setClimbSetPoint(double setPoint){
        climbSetPoint = setPoint;
    }

    public double getClimbSetPoint(){
        return climbSetPoint;
    }

    public void closedLoopClimb(){
        double sp = getClimbSetPoint();
        climbClosedLoopController.setReference(sp, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    public boolean isAtPosition() {
        double error = getPosition() - climbSetPoint;
        return (Math.abs(error) < allowableError);
      }

    public void climbOut(){
        this.setClimbSetPoint(-290);
    }

    
 }

