package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeCoralSubsystem;

public class IntakeCmd extends Command {

    IntakeCoralSubsystem intake;
    boolean finishes;

    public IntakeCmd(IntakeCoralSubsystem intake){
        this.intake = intake;
        this.finishes = true;
        addRequirements(intake);
    }

    @Override
  public void execute() {
    this.intake.intake();
    if(intake.getCurrent() >= 30){
        finishes = true;
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      if (finishes) {
      return true;
    } else {
      return false;
    }
  }
}
