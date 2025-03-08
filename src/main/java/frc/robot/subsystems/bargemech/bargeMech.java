// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.bargemech;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class bargeMech extends SubsystemBase {

  private final bargeIO barge;

  /** Creates a new EndEffector. */
  public bargeMech(bargeIO b) {
    this.barge = b;
  }

  public void intake(double speed) {}

  public void outake(double speed) {}

  public void stopBarge() {
    this.barge.stopMotor();
  }

  @Override
  public void periodic() {}
}
