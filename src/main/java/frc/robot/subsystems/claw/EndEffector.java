// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.claw;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EndEffector extends SubsystemBase {

  private final ClawIO claw;
  private final WristIO wrist;

  /** Creates a new EndEffector. */
  public EndEffector(ClawIO clawimpl, WristIO wristimpl) {
    this.claw = clawimpl;
    this.wrist = wristimpl;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
