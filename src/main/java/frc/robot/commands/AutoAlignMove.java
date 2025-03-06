// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.util.OffsetDirection;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlignMove extends Command {
    CommandSwerveDrivetrain driveTrain;
    Optional<Alliance> alliance;
    Pose2d desiredPosition;
    private final PIDController x_controller = new PIDController(10, 0, 0);
      private final PIDController y_controller = new PIDController(10, 0,0);
      private final PIDController theta_controller = new PIDController(20, 0, 0);
      private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();
      OffsetDirection offsetDirection = OffsetDirection.CENTER;

  /** Creates a new AutoAlignMove. */
  public AutoAlignMove(CommandSwerveDrivetrain drivetrain, Optional<Alliance> allianceColor, Pose2d DesiredPosition, OffsetDirection offset) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = drivetrain;
    this.alliance = allianceColor;
    this.desiredPosition = DesiredPosition;
    this.offsetDirection = offset;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(this.alliance.get() == Alliance.Red){
        driveTrain.applyRequest(() -> 
        drive.withVelocityX(x_controller.calculate(driveTrain.getState().Pose.getX(),desiredPosition.getX())) // Drive forward with negative Y (forward)
            .withVelocityY(y_controller.calculate(driveTrain.getState().Pose.getY(),desiredPosition.getY())) // Drive left with negative X (left)
            .withRotationalRate(theta_controller.calculate(driveTrain.getState().Pose.getRotation().getRadians(),desiredPosition.getRotation().getRadians())));
      }

      if(this.alliance.get() == Alliance.Blue){
        driveTrain.applyRequest(() -> 
        drive.withVelocityX(-x_controller.calculate(driveTrain.getState().Pose.getX(),desiredPosition.getX())) // Drive forward with negative Y (forward)
            .withVelocityY(-y_controller.calculate(driveTrain.getState().Pose.getY(),desiredPosition.getY())) // Drive left with negative X (left)
            .withRotationalRate(theta_controller.calculate(driveTrain.getState().Pose.getRotation().getRadians(),desiredPosition.getRotation().getRadians())));
      }
      
    }



  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
