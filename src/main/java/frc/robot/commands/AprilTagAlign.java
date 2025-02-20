// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.path.IdealStartingState;

import static edu.wpi.first.units.Units.Kilo;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.lang.annotation.Target;
import java.time.OffsetDateTime;
import java.time.OffsetTime;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.OffsetDirection;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AprilTagAlign extends Command {
  /** Creates a new AprilTagAlign. */
      //SwerveRequest.FieldCentric drive;
      CommandSwerveDrivetrain drivetrain;
      Pose2d currentPosition;
      int targetNumber = 0;
      double lengthFromTarget = 1.0;
      double offestFromTarget = .2;
      OffsetDirection offsetDirection = OffsetDirection.CENTER;
      int[] targets = {6,7,8,9,10,11,17,18,19,22};





      double kDt = .02;
  
  
      private final TrapezoidProfile.Constraints x_constraints = new TrapezoidProfile.Constraints(1, 2);
      private final ProfiledPIDController x_controller = new ProfiledPIDController(1.0 , 0, 0, x_constraints, kDt);
      private final TrapezoidProfile.Constraints y_constraints = new TrapezoidProfile.Constraints(1, 2);
      private final ProfiledPIDController y_controller = new ProfiledPIDController(1.0 , 0, 0, y_constraints, kDt);
      private final TrapezoidProfile.Constraints theta_constraints = new TrapezoidProfile.Constraints(3.14, 6.2);
      private final ProfiledPIDController theta_controller = new ProfiledPIDController(1.0 , 0, 0, theta_constraints, kDt);

      Pose2d desiredPosition = new Pose2d();

      private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
      private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

      private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  
      

  
  public AprilTagAlign(CommandSwerveDrivetrain drivetrain, OffsetDirection direction){ // Use open-loop control for drive motors) {
    // Use addRequirements() here to declare subsystem dependencies.
    
    this.drivetrain = drivetrain;
    this.offsetDirection = direction;


    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    resetController();
    //double tagNumber = LimelightHelpers.getFiducialID("limelight");
    double tagNumber = 11.0;
    this.targetNumber = (int)Math.round(tagNumber); //Makes Fiducial ID a integer instead of a network tables double
    //boolean isLLTargetDetected = LimelightHelpers.getTV("limelight");
    boolean isLLTargetDetected = true;
    if(isLLTargetDetected){
      if(isTargetInList(targetNumber, targets)){
        desiredPosition = getTargetPosition(targetNumber, lengthFromTarget, offestFromTarget, offsetDirection);
      }
      
      
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentPosition = drivetrain.getState().Pose;
    SmartDashboard.putNumber("Target X", desiredPosition.getX());
    SmartDashboard.putNumber("Target Y", desiredPosition.getY());
    SmartDashboard.putNumber("Target Rot",desiredPosition.getRotation().getDegrees());
    SmartDashboard.putNumber("Current X",currentPosition.getX());
    SmartDashboard.putNumber("Current Y", currentPosition.getY());
    SmartDashboard.putNumber("Current Rot", currentPosition.getRotation().getDegrees());

    

    double xOutput = x_controller.calculate(currentPosition.getX(), desiredPosition.getX());
    double yOutput = y_controller.calculate(currentPosition.getY(), desiredPosition.getY());
    double thetaOuput = theta_controller.calculate(currentPosition.getRotation().getRadians(), desiredPosition.getRotation().getRadians());
    SmartDashboard.putNumber("X Ouptut", xOutput);
    SmartDashboard.putNumber("Y Output", yOutput);
    SmartDashboard.putNumber("Theta Output", thetaOuput);

    drivetrain.applyRequest(() ->
                 drive.withVelocityX(xOutput) // Drive forward with negative Y (forward)
                     .withVelocityY(yOutput) // Drive left with negative X (left)
                     .withRotationalRate((thetaOuput)) // Drive counterclockwise with negative X (left)
             );
  
    


    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }




  public void resetController(){
    x_controller.reset(drivetrain.getState().Pose.getX());
    y_controller.reset(drivetrain.getState().Pose.getY());
    theta_controller.reset(drivetrain.getState().Pose.getRotation().getRadians());
  }



public boolean isTargetInList(int targetId, int[] targets){
  
  boolean isTarget = false;
  int i;

  for(i = 0; i<targets.length; i++){
    if(targetId == targets[i] ){
      isTarget = true;
    }
  }


  return isTarget;
}  
  
      

      
      
public Pose2d getTargetPosition(int target, double lengthFromTarget, double offsetFromTargetCenter, OffsetDirection offset){

      double x_inches = 0.0;
      double y_inches = 0.0;
      double theta_deg = 0.0;
        
        switch (target){
          case 6:
            x_inches = 530.49;
            y_inches = 130.17;
            theta_deg = 300.0;
            break;

          case 7:
                x_inches = 546.87;
                y_inches = 158.50;
                theta_deg = 0.0;
                break;
          case 8:
            x_inches = 530.49;
            y_inches = 186.83;
            theta_deg = 60.0;

          case 9:
            x_inches = 497.77;
            y_inches = 186.83;
            theta_deg = 120.0;
          case 10:
            x_inches = 481.39;
            y_inches = 158.50;
            theta_deg = 180.0;
          case 11:
            x_inches = 497.77;
            y_inches = 130.17;
            theta_deg = 240.0;
          case 17:
            x_inches = 160.39;
            y_inches = 130.17;
            theta_deg = 240.0;
              break;
          case 18:
            x_inches = 144.0;
            y_inches = 158.5;
            theta_deg = 180.0;
            break;
          case 19:
            x_inches = 160.39;
            y_inches = 186.83;
            theta_deg = 120;
            break;
          case 20:
            x_inches = 193.1;
            y_inches = 186.83;
            theta_deg = 60;
            break;
          case 21:
            x_inches = 209.49;
            y_inches = 158.5;
            theta_deg = 0;
            break;
          case 22:
            x_inches = 193.1;
            y_inches = 130.17;
            theta_deg = 300.0;
            break;
          default:
            x_inches = 0;
            y_inches = 0;
            theta_deg = 0;
      
         }
          
          double x_meters = x_inches*25.4/1000;
          double y_meters = y_inches*25.4/1000;
          double rad = theta_deg*Math.PI/180;
          
          
          double dx = lengthFromTarget * Math.cos(rad);
          double dy = lengthFromTarget * Math.sin(rad);
          
          double x1 = x_meters + dx; //positoin directly in front of target that is lengthFromTarget Away
          double y1 = y_meters + dy;
          
          double dax1 = offsetFromTargetCenter * Math.sin(2*Math.PI-rad);
          double day1 = offsetFromTargetCenter * Math.cos(2*Math.PI-rad);
          
          double ax1 = x1 + dax1; //positoin right of target
          double ay1 = y1 + day1;
          
          double ax2 = x1 - dax1;  //position left of target
          double ay2 = y1 - day1;

          double x = 0.0;
          double y = 0.0;
          double phi = (theta_deg + 180.0)%360.0;
          double phi_rad = phi*Math.PI/180.0;


          
          switch (offset){
            case CENTER:
              x = x1;
              y = y1;
              
                break;
            case RIGHT:
              x = ax1;
              y = ay1;
              
              break;
            case LEFT:
              x = ax2;
              y = ay2;
              
              break;

          }
            
          
          Pose2d targtetPose = new Pose2d(x,y, new Rotation2d(phi_rad));
        
         
          
          
    return targtetPose;
    
    }

    
    

}
