// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.enums;

import edu.wpi.first.math.geometry.Pose2d;

/** Add your docs here. */
public enum TargetPose {
  NONE(new Pose2d()),
  TWELVERIGHT(new Pose2d()),
  TWELVELEFT(new Pose2d()),
  TWORIGHT(new Pose2d()),
  TWOLEFT(new Pose2d()),
  FOURRIGHT(new Pose2d()),
  FOURLEFT(new Pose2d()),
  SIXRIGHT(new Pose2d()),
  SIXLEFT(new Pose2d()),
  EIGHTRIGHT(new Pose2d()),
  EIGHTLEFT(new Pose2d()),
  TENRIGHT(new Pose2d()),
  TENLEFT(new Pose2d());

  private Pose2d targetPose;

  public Pose2d getTargetPose() {
    return targetPose;
  }

  private TargetPose(Pose2d pose) {
    this.targetPose = pose;
  }
}
