// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  private final ElevatorIONeo elevator;

  private elevatorState state = elevatorState.DEFAULT;
  ;

  public enum elevatorState {
    DEFAULT,
    INTAKE,
    L1,
    L2,
    L3,
    L4
  }

  public Elevator() {
    elevator = new ElevatorIONeo();
  }

  public void setStateIntake() {
    state = elevatorState.INTAKE;
  }

  public void setStateL1() {
    state = elevatorState.L1;
  }

  public void setStateL2() {
    state = elevatorState.L2;
  }

  public void setStateL3() {
    state = elevatorState.L3;
  }

  public void setStateL4() {
    state = elevatorState.L4;
  }

  @Override
  public void periodic() {

    switch (state) {
      case DEFAULT:
        elevator.moveToPoint(RobotConstants.Elevator.defaultheight);
        break;

      case INTAKE:
        elevator.moveToPoint(RobotConstants.Elevator.intakeheight);
        break;

      case L1:
        elevator.moveToPoint(RobotConstants.Elevator.L1height);
        break;

      case L2:
        elevator.moveToPoint(RobotConstants.Elevator.L2height);
        break;

      case L3:
        elevator.moveToPoint(RobotConstants.Elevator.L3height);
        break;

      case L4:
        elevator.moveToPoint(RobotConstants.Elevator.L4height);
        break;

      default:
        elevator.moveToPoint(RobotConstants.Elevator.defaultheight);
        break;
    }
  }
}
