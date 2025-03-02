// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.StateSet.IntakeStates.intakeCenterBackwardState;
import frc.robot.commands.StateSet.IntakeStates.intakeCenterForwardState;
import frc.robot.commands.StateSet.IntakeStates.intakeState;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.statehandler.StateHandler;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Intake extends SequentialCommandGroup {

  public Intake(StateHandler stateHandler, EndEffector ee, Elevator e, CommandXboxController controller) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new intakeState(stateHandler, ee), new InstantCommand(), new intakeCenterForwardState(stateHandler, ee), new intakeCenterBackwardState(stateHandler, ee));
  }
}
