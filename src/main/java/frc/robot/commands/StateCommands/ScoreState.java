package frc.robot.commands.StateCommands;

import edu.wpi.first.wpilibj2.command.Command;

public class ScoreState extends Command {

  public ScoreState() {
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements();
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {}

  @Override
  public boolean isFinished() {
    // TODO: Make this return true when this Command no longer needs to run execute()
    return false;
  }

  @Override
  public void end(boolean interrupted) {}
}
