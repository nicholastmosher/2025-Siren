// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants.Elevator.elevatorState;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  private final ElevatorIO elevator;

  private ElevatorIOInputsAutoLogged elevatorinputs;

  private elevatorState state;

  public Elevator(ElevatorIO elevator) {
    this.elevator = elevator;
    elevatorinputs = new ElevatorIOInputsAutoLogged();
    this.state = elevatorState.DEFAULT;
  }

  public void moveElevator(double input) {
    double currentState = this.elevator.getEncoder();

    double targetState = currentState+(input*2);

    this.elevator.moveToPoint(Rotation2d.fromDegrees(targetState));
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

  public ElevatorIO getElevatorInstance() {
    return this.elevator;
  }

  @Override
  public void periodic() {
    elevator.moveToState(state);
    elevator.updateInputs(elevatorinputs);
  }
}
