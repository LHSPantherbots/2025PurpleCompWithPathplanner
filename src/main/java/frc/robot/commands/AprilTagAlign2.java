// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.OffsetDirection;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AprilTagAlign2 extends Command {
  /** Creates a new AprilTagAlign. */
      

  CommandSwerveDrivetrain drivetrain;
  Object[][] targetLocations = {{1, 16.3934,	1.0327, 306.0},  // blue origin location .5 meters in fron of every relevent target
                                {2, 16.3934,	7.0098, 54.0},
                                {3, 11.4910,	7.5317, 90.0},
                                {6, 13.7244, 2.8682, 120.0},
                                {7, 14.3905, 4.0208,	180.0},
                                {8, 13.7244,	5.1734,	240.0},
                                {9, 12.3934,	5.1734,	300.0},
                                {10, 11.7273,	4.0208,	0.0},
                                {11, 12.3934,	2.8682,	60.0},
                                {12, 1.1552,	1.0327,	234.0},
                                {13, 1.1552,	7.0098,	126.0},
                                {16, 6.0576,	0.5107,	270.0000},
                                {17, 3.8239,	2.8733,	60.0},
                                {18, 3.1576,	4.0259,	0.0000},
                                {19, 3.8239,	5.1785,	300.0},
                                {20, 5.1547,	5.1785,	240.0},
                                {21, 5.8210,	4.0259,	180.0},
                                {22, 5.1547,	2.8733,	120.0}
                                
};
  
  
  
  
  //SwerveRequest.FieldCentric drive;

      Pose2d currentPosition;
      int targetNumber = 0;
      double lengthFromTarget = .6985;
      double offestFromTarget = .1651;
      OffsetDirection offsetDirection = OffsetDirection.CENTER;
      int[] targets = {1,2,3,6,7,8,9,10,11,12,13,16,17,18,19,20,21,22};


      Pose2d desiredPosition = new Pose2d();



  
  public AprilTagAlign2(CommandSwerveDrivetrain drivetrain, OffsetDirection offset){ // Use open-loop control for drive motors) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.offsetDirection = offset;

    this.drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //resetController();
    //double tagNumber = LimelightHelpers.getFiducialID("limelight");
    currentPosition = drivetrain.getState().Pose;
    double tagNumber = getClosestTargetID(currentPosition);
    this.targetNumber = (int)Math.round(tagNumber); //Makes Fiducial ID a integer instead of a network tables double

    desiredPosition = drivetrain.getState().Pose;
    
      if(isTargetInList(targetNumber, targets)){
        desiredPosition = getTargetPosition(targetNumber, lengthFromTarget, offestFromTarget, offsetDirection);
      }
      RobotContainer.desiredPosition=desiredPosition;
      
      
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    

    

      


    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
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
  
public int getClosestTargetID(Pose2d currentPosition){
  double minDistance = 1000.0;
  int closestTarget = 0;
  double x1 = currentPosition.getX();
  double y1 = currentPosition.getY();



  for (int i = 0; i<targetLocations.length; i++){
    int currentTarget = (int)targetLocations[i][0];
    double x2 = (double)targetLocations[i][1];
    double y2 = (double)targetLocations[i][2];
    double distance = Math.sqrt(Math.pow((x1-x2), 2)+Math.pow((y1-y2),2));
    if(distance<minDistance){
      closestTarget = currentTarget;
      minDistance = distance;

    }

  } 
  System.out.println("Closest Target" + closestTarget);

  return closestTarget;
}

      
      
public Pose2d getTargetPosition(int target, double lengthFromTarget, double offsetFromTargetCenter, OffsetDirection offset){

      double x_inches = 0.0;
      double y_inches = 0.0;
      double theta_deg = 0.0;
        
        switch (target){
          case 1:
            x_inches = 656.98;
            y_inches = 24.73;
            theta_deg = 126.0;

          break;
          case 2:
            x_inches = 656.98;
            y_inches = 291.90;
            theta_deg = 234.0;

          break;
          case 3:

            x_inches = 452.40;
            y_inches = 316.21;
            theta_deg = 270;

          break;
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
            break;

          case 9:
            x_inches = 497.77;
            y_inches = 186.83;
            theta_deg = 120.0;
            break;
          case 10:
            x_inches = 481.39;
            y_inches = 158.50;
            theta_deg = 180.0;
            break;
          case 11:
            x_inches = 497.77;
            y_inches = 130.17;
            theta_deg = 240.0;
            break;
          case 12:
            x_inches = 33.91;
            y_inches = 24.73;
            theta_deg = 54.0;
            break;
          case 13:
            x_inches = 33.91;
            y_inches = 291.90;
            theta_deg = 306.0;
            break;
          case 16:
            x_inches = 238.49;
            y_inches = .42;
            theta_deg = 90;
            break;
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
