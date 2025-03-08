// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.bargemech;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.virtualsubsystems.statehandler.StateHandler;
import org.littletonrobotics.junction.Logger;

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
  public void periodic() {

  }
}
