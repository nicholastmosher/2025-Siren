// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  private final ElevatorIO elevator;

  private ElevatorIOInputsAutoLogged elevatorinputs;

  private elevatorState state = elevatorState.DEFAULT;

  public Elevator() {
    elevator = new ElevatorIONeo();
    elevatorinputs = new ElevatorIOInputsAutoLogged();
  }

  @Override
  public void periodic() {

    elevator.updateInputs(elevatorinputs);
  }
}
