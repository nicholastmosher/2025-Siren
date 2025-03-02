// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.statehandler;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.enums.robotstate;
import org.littletonrobotics.junction.Logger;

public class StateHandler extends SubsystemBase {
  /** Creates a new StateHandler. */
  robotstate state;

  public StateHandler() {

    this.state = robotstate.RESTING;
  }

  public void setState(robotstate targetState) {
    this.state = targetState;
  }

  public robotstate getState() {
    return this.state;
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) {
      this.state = robotstate.STOP;
    }

    Logger.recordOutput("StateHandler/state", this.state.toString());
  }
}
