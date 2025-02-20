// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class TargetPositionFinder {

    Pose2d currentPosition;
    int targetNumber = 0;
    double lengthFromTarget = 1.0;
    double offestFromTarget = .2;
    OffsetDirection offsetDirection = OffsetDirection.CENTER;
    int[] targets = {6,7,8,9,10,11,17,18,19,20,21,22};
    Pose2d desiredPosition = new Pose2d();

    double tagNumber = 20.0;


    public TargetPositionFinder(){
        this.targetNumber = (int)Math.round(tagNumber); //Makes Fiducial ID a integer instead of a network tables double
        //boolean isLLTargetDetected = LimelightHelpers.getTV("limelight");
        boolean isLLTargetDetected = true;
        if(isLLTargetDetected){
          if(isTargetInList(targetNumber, targets)){
            desiredPosition = getTargetPosition(targetNumber, lengthFromTarget, offestFromTarget, offsetDirection);
            RobotContainer.desiredPosition=desiredPosition;;
          }
          
          
        }
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
