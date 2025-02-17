// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.claw;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EndEffector extends SubsystemBase {

  private final ClawIO claw;
  private final WristIO wrist;

  /** Creates a new EndEffector. */
  public EndEffector(ClawIO clawimpl, WristIO wristimpl) {
    this.claw = clawimpl;
    this.wrist = wristimpl;
  }

  public void outEndEffector(double speed) {
    this.claw.setSpeed(speed);
  }

  public void inEndEffector(double speed) {
    this.claw.setSpeed(speed);
  }

  public void setWristAngle(Rotation2d angle) {
    this.wrist.setAngle(angle);
  }

  public void stopWrist() {}

  public void stopClaw() {
    this.claw.stopMotor();
  }

  @Override
  public void periodic() {
    this.claw.updateInputs();
    this.wrist.updateInputs();
  }
}
