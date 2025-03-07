// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.enums;

import frc.lib.config.robotstateconfig;
import frc.lib.constants.RobotConstants.ElevatorConstants;
import frc.lib.constants.RobotConstants.EndEffectorConstants;
import frc.lib.constants.RobotConstants.GroundIntakeConstants;

/** Add your docs here. */
public enum robotStates {
  STOP(
      new robotstateconfig(
          ElevatorConstants.defaultheight,
          EndEffectorConstants.defaultrot,
          GroundIntakeConstants.defaultangle,
          GroundIntakeConstants.staticSpeed,
          EndEffectorConstants.staticSpeed,
          TargetPose.NONE)),
  RESTING(
      new robotstateconfig(
          ElevatorConstants.defaultheight,
          EndEffectorConstants.defaultrot,
          GroundIntakeConstants.defaultangle,
          GroundIntakeConstants.staticSpeed,
          EndEffectorConstants.staticSpeed,
          TargetPose.NONE)),
  INTAKE(
      new robotstateconfig(
          ElevatorConstants.defaultheight,
          EndEffectorConstants.defaultrot,
          GroundIntakeConstants.defaultangle,
          GroundIntakeConstants.staticSpeed,
          EndEffectorConstants.staticSpeed,
          TargetPose.NONE,
          false)),
  INTAKECENTERFORWARD(
      new robotstateconfig(
          ElevatorConstants.defaultheight,
          EndEffectorConstants.defaultrot,
          GroundIntakeConstants.defaultangle,
          GroundIntakeConstants.staticSpeed,
          EndEffectorConstants.staticSpeed,
          TargetPose.NONE)),
  INTAKECENTERBACKWARD(
      new robotstateconfig(
          ElevatorConstants.defaultheight,
          EndEffectorConstants.defaultrot,
          GroundIntakeConstants.defaultangle,
          GroundIntakeConstants.staticSpeed,
          EndEffectorConstants.staticSpeed,
          TargetPose.NONE)),
  L1PREPARE(
      new robotstateconfig(
          ElevatorConstants.defaultheight,
          EndEffectorConstants.defaultrot,
          GroundIntakeConstants.defaultangle,
          GroundIntakeConstants.staticSpeed,
          EndEffectorConstants.staticSpeed,
          TargetPose.NONE)),
  L1SCORE(
      new robotstateconfig(
          ElevatorConstants.defaultheight,
          EndEffectorConstants.defaultrot,
          GroundIntakeConstants.defaultangle,
          GroundIntakeConstants.staticSpeed,
          EndEffectorConstants.staticSpeed,
          TargetPose.NONE)),
  L2PREPARE(
      new robotstateconfig(
          ElevatorConstants.defaultheight,
          EndEffectorConstants.defaultrot,
          GroundIntakeConstants.defaultangle,
          GroundIntakeConstants.staticSpeed,
          EndEffectorConstants.staticSpeed,
          TargetPose.NONE)),
  L2SCORE(
      new robotstateconfig(
          ElevatorConstants.defaultheight,
          EndEffectorConstants.defaultrot,
          GroundIntakeConstants.defaultangle,
          GroundIntakeConstants.staticSpeed,
          EndEffectorConstants.staticSpeed,
          TargetPose.NONE)),
  L3PREPARE(
      new robotstateconfig(
          ElevatorConstants.defaultheight,
          EndEffectorConstants.defaultrot,
          GroundIntakeConstants.defaultangle,
          GroundIntakeConstants.staticSpeed,
          EndEffectorConstants.staticSpeed,
          TargetPose.NONE)),
  L3SCORE(
      new robotstateconfig(
          ElevatorConstants.defaultheight,
          EndEffectorConstants.defaultrot,
          GroundIntakeConstants.defaultangle,
          GroundIntakeConstants.staticSpeed,
          EndEffectorConstants.staticSpeed,
          TargetPose.NONE)),
  L4PREPARE(
      new robotstateconfig(
          ElevatorConstants.defaultheight,
          EndEffectorConstants.defaultrot,
          GroundIntakeConstants.defaultangle,
          GroundIntakeConstants.staticSpeed,
          EndEffectorConstants.staticSpeed,
          TargetPose.NONE)),
  L4SCORE(
      new robotstateconfig(
          ElevatorConstants.defaultheight,
          EndEffectorConstants.defaultrot,
          GroundIntakeConstants.defaultangle,
          GroundIntakeConstants.staticSpeed,
          EndEffectorConstants.staticSpeed,
          TargetPose.NONE)),
  GROUNDINTAKE(
      new robotstateconfig(
          ElevatorConstants.defaultheight,
          EndEffectorConstants.defaultrot,
          GroundIntakeConstants.defaultangle,
          GroundIntakeConstants.staticSpeed,
          EndEffectorConstants.staticSpeed,
          TargetPose.NONE)),
  GROUNDHOLD(
      new robotstateconfig(
          ElevatorConstants.defaultheight,
          EndEffectorConstants.defaultrot,
          GroundIntakeConstants.defaultangle,
          GroundIntakeConstants.staticSpeed,
          EndEffectorConstants.staticSpeed,
          TargetPose.NONE));

  private robotstateconfig targetrobotstate;

  public robotstateconfig getRobotStateConfig() {
    return targetrobotstate;
  }

  private robotStates(robotstateconfig targetstate) {
    this.targetrobotstate = targetstate;
  }
}
