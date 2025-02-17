// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AlgaeL2;
import frc.robot.commands.AlgaeL3;
import frc.robot.commands.AlgaeStowAll;
import frc.robot.commands.AutoIntakeCmd;
import frc.robot.commands.CorralIntake;
import frc.robot.commands.CorralScoreL1Dump;
import frc.robot.commands.CorralScoreL2;
import frc.robot.commands.CorralScoreL3;
import frc.robot.commands.CorralScoreL4;
import frc.robot.commands.CorralScoreL4Flip;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeAlgaeSubsystem;
import frc.robot.subsystems.IntakeCoralSubsystem;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.util.Position;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
  
    


    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

   // Replace with CommandPS4Controller or CommandJoystick if needed
     private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
    private final CommandXboxController m_operatorController = 
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);
   
      public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final WristSubsystem wrist = new WristSubsystem();
    private final IntakeCoralSubsystem coral = new IntakeCoralSubsystem();
    private final IntakeAlgaeSubsystem algae = new IntakeAlgaeSubsystem();
    public static Leds leds = new Leds();
    private final ElevatorSubsystem elevator = new ElevatorSubsystem();
    private final ClimbSubsystem climb = new ClimbSubsystem();



    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        NamedCommands.registerCommand("dumpL1", new CorralScoreL1Dump(wrist, elevator));


        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
   //DEFAULT COMMANDS


    drivetrain.setDefaultCommand(
    //         // Drivetrain will execute this command periodically
             drivetrain.applyRequest(() ->
                 drive.withVelocityX(-m_driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                     .withVelocityY(-m_driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                     .withRotationalRate((m_driverController.getLeftTriggerAxis()-m_driverController.getRightTriggerAxis()) * MaxAngularRate) // Drive counterclockwise with negative X (left)
             )
         );

    elevator.setDefaultCommand(
        new RunCommand(
          () -> elevator.motionMagicSetPosition(), elevator));
          //.manualDrive(-m_driverController.getRightY()*.25), elevator));

    climb.setDefaultCommand(
      new RunCommand(
        ()-> climb.manualClimbMove(0.0), climb));

    wrist.setDefaultCommand(
            new RunCommand(() -> wrist.closedLoopWrist(), wrist));
    
    coral.setDefaultCommand(new RunCommand(() -> coral.intakeStop(), coral));
    algae.setDefaultCommand(new RunCommand(() -> algae.intakeStop(), algae));
    leds.setDefaultCommand(new RunCommand(() -> leds.ledState(), leds));




    //************  DRIVER CONTROLLER  ****************

    m_driverController.leftBumper().whileTrue(
        new RunCommand(
            () -> climb.manualClimbMove(-MathUtil.applyDeadband(m_driverController.getRightY(), OperatorConstants.kDriveDeadband)),
            climb));
    
    m_driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
    
    //PROBABLY REMOVE THIS ONE
    m_driverController.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-m_driverController.getLeftY(), -m_driverController.getLeftX()))
    ));

    //Slow Mode
    m_driverController.rightBumper().whileTrue(drivetrain.applyRequest(() ->
    drive.withVelocityX(-m_driverController.getLeftY() * MaxSpeed * .25) // Drive forward with negative Y (forward)
        .withVelocityY(-m_driverController.getLeftX() * MaxSpeed * .25) // Drive left with negative X (left)
        .withRotationalRate((m_driverController.getLeftTriggerAxis()-m_driverController.getRightTriggerAxis()) * MaxAngularRate * .25)));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    m_driverController.back().and(m_driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    
    m_driverController.back().and(m_driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    
    m_driverController.start().and(m_driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    
    m_driverController.start().and(m_driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // reset the field-centric heading on start press
    m_driverController.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    drivetrain.registerTelemetry(logger::telemeterize);



    //************   OPERATOR CONTROLLER  *******************

    m_operatorController.povDown().onTrue(new CorralIntake(wrist, elevator));  //Intake/Stow
    
    m_operatorController.povLeft().onTrue(new CorralScoreL2(wrist, elevator));

    m_operatorController.povUp().onTrue(new CorralScoreL3(wrist, elevator));

    m_operatorController.povRight().onTrue(new CorralScoreL4(wrist, elevator));

    if (leds.getRobotStatus() == Position.CORAL_L4){ ////This is triying to flip the coral onto L4
         m_operatorController.rightBumper().whileTrue(new CorralScoreL4Flip(wrist,elevator,coral));
    }else{
         m_operatorController.rightBumper().whileTrue(new RunCommand(() -> coral.intake(), coral));
    }

    //m_operatorController.rightBumper().onTrue(new AutoIntakeCmd(coral));

    
    m_operatorController.leftBumper().whileTrue(new RunCommand(() -> coral.outtake(), coral));

    //m_operatorController.leftBumper().onTrue(new RunCommand(() -> coral.outtake(), coral).withTimeout(.25));


    m_operatorController.a().whileTrue(new RunCommand(()-> algae.intake(), algae));
    
    m_operatorController.x().whileTrue(new RunCommand(()-> algae.outtake(), algae));

    m_operatorController.leftTrigger().whileTrue(new RunCommand(() -> wrist.manualWristMove(-m_operatorController.getRightY()*.25), wrist));

    m_operatorController.y().onTrue(new AlgaeL3(wrist, elevator));

    m_operatorController.b().onTrue(new AlgaeL2(wrist, elevator));

    m_operatorController.start().onTrue(new AlgaeStowAll(wrist, elevator));


    






    drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
