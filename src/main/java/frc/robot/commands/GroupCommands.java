package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.enums.LevelEnum;
import frc.robot.ToggleHandler;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.groundintake.GroundIntake;
import frc.robot.subsystems.virtualsubsystems.statehandler.StateHandler;

public class GroupCommands {
  public static Command intake(
      Drive drive,
      Elevator elevator,
      EndEffector endEffector,
      GroundIntake groundIntake,
      StateHandler stateHandler,
      ToggleHandler elevatorDisable) {
    return Commands.sequence(
        Commands.runOnce(() -> stateHandler.setLevelEnum(LevelEnum.INTAKE)),
        StateCommands.elevatorToChosenHeight(elevator, endEffector, stateHandler, elevatorDisable),
        StateCommands.intake(elevator, endEffector, groundIntake, stateHandler),
        StateCommands.intakeCenterForward(elevator, endEffector, groundIntake, stateHandler),
        StateCommands.intakeCenterBackward(elevator, endEffector, groundIntake, stateHandler));
  }

  public static Command score(
      Drive drive,
      Elevator elevator,
      EndEffector endEffector,
      GroundIntake groundIntake,
      StateHandler stateHandler,
      ToggleHandler elevatorDisable,
      ToggleHandler alignDisable) {
    return Commands.sequence(
        Commands.parallel(
            new ToClosestReefPoseCommand(drive, alignDisable),
            StateCommands.elevatorToChosenHeight(
                elevator, endEffector, stateHandler, elevatorDisable)),
        StateCommands.placeAtChosenHeight(elevator, endEffector, stateHandler, elevatorDisable)
            .withTimeout(1),
        Commands.parallel(
            DriveCommands.driveBackwards(drive).withTimeout(0.8),
            StateCommands.restingState(elevator, endEffector, stateHandler)));
  }
}
