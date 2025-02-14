// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants.Elevator.elevatorState;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  private final ElevatorIO elevator;

  private ElevatorIOInputsAutoLogged elevatorinputs;

  private elevatorState state;

  public Elevator() {
    this.elevator = new ElevatorIONeo();
    elevatorinputs = new ElevatorIOInputsAutoLogged();
    this.state = elevatorState.DEFAULT;
  }

  public void setGoalState(elevatorState givenstate) {
    this.state = givenstate;
  }

  public elevatorState getGoalState() {
    return this.state;
  }

  public void stopElevator() {
    elevator.stopElevator();
  }

  @Override
  public void periodic() {
    elevator.moveToState(state);
    elevator.updateInputs(elevatorinputs);
  }
}
