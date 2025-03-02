// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.config;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.enums.TargetPose;

/** Add your docs here. */
public class robotstateconfig {

  public final Rotation2d elevatorheight;
  public final Rotation2d wristRotation;
  public final Rotation2d groundIntakeRot;
  public final double groundIntakeSpeed;
  public final double clawSpeed;
  public final TargetPose targetPose;

  public robotstateconfig(
      Rotation2d elevatorRot,
      Rotation2d wristRot,
      Rotation2d groundIntakeRot,
      double groundIntakeSpeed,
      double clawSpeed,
      TargetPose targetPose) {
    this.elevatorheight = elevatorRot;
    this.wristRotation = wristRot;
    this.groundIntakeRot = groundIntakeRot;
    this.groundIntakeSpeed = groundIntakeSpeed;
    this.clawSpeed = clawSpeed;
    this.targetPose = targetPose;
  }
}
