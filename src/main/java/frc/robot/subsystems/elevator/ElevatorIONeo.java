// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class ElevatorIONeo implements ElevatorIO {

  private final SparkMax leadMotor;
  private final SparkMax followerMotor;
  private final SparkAbsoluteEncoder elevatorEncoder;

  // private final StatusSignal<Rotation2d> rotations =
  // Rotation2d.fromDegrees(elevatorEncoder.getPosition());

  public ElevatorIONeo() {
    leadMotor = new SparkMax(0, MotorType.kBrushless);

    followerMotor = new SparkMax(0, MotorType.kBrushless);

    elevatorEncoder = leadMotor.getAbsoluteEncoder();
  }

  public void moveElevator(double height) {}

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.leadMotorConnected = true;
    inputs.followerMotorConnected = true;
    inputs.encoderConnected = true;

    inputs.elevatorState = Rotation2d.fromDegrees(elevatorEncoder.getPosition());
  }
}
