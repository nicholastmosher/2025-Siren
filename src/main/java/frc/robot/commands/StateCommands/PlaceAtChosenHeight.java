package frc.robot.commands.StateCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.enums.robotStates;
import frc.robot.ToggleHandler;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.virtualsubsystems.statehandler.StateHandler;

public class PlaceAtChosenHeight extends Command {
  private final Elevator elevator;
  private final EndEffector endEffector;
  private final StateHandler stateHandler;
  private final ToggleHandler disable;

  public PlaceAtChosenHeight(
      Elevator elevator, EndEffector endEffector, StateHandler handler, ToggleHandler disable) {
    this.elevator = elevator;
    this.endEffector = endEffector;
    this.stateHandler = handler;
    this.disable = disable;
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.elevator, this.endEffector);
  }

  @Override
  public void initialize() {
    switch (this.stateHandler.getChosenlevel()) {
      case L1 -> this.stateHandler.setState(robotStates.L1SCORE);
      case L2 -> this.stateHandler.setState(robotStates.L2SCORE);
      case L3 -> this.stateHandler.setState(robotStates.L3SCORE);
      case L4 -> this.stateHandler.setState(robotStates.L4SCORE);
    }
  }

  @Override
  public void execute() {}

  @Override
  public boolean isFinished() {
    return this.disable.get();
  }

  @Override
  public void end(boolean interrupted) {
    if (disable.get()) {
      this.elevator.stopElevator();
      return;
    }
    this.stateHandler.setState(robotStates.RESTING);
  }
}
