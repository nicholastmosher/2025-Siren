// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ElevatorIO {

  @AutoLog
  public static class ElevatorIOInputs {
    public boolean leadMotorConnected;
    public boolean followerMotorConnected;
    public boolean encoderConnected;

    public Rotation2d elevatorState;
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}
}
