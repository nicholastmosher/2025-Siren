// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.StateSet.L2States;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.enums.robotstate;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.statehandler.StateHandler;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class L2Score extends Command {
  /** Creates a new L1Prepare. */
  private final StateHandler stateHandler;

  private final Elevator elevator;
  private final EndEffector endEffector;

  public L2Score(StateHandler handler, Elevator e, EndEffector ee) {
    this.stateHandler = handler;
    this.elevator = e;
    this.endEffector = ee;
    addRequirements(this.stateHandler, this.elevator, this.endEffector);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.stateHandler.setState(robotstate.L2SCORE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !this.endEffector.getfrontIntaked();
  }
}
