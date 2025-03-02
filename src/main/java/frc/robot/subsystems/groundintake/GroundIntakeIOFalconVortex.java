// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.groundintake;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.constants.RobotConstants.GroundIntakeConstants;

/** Add your docs here. */
public class GroundIntakeIOFalconVortex implements GroundIntakeIO {

  private final TalonFX tiltMotor;
  private final SparkFlex spinMotor;

  public GroundIntakeIOFalconVortex() {

    tiltMotor = new TalonFX(GroundIntakeConstants.tiltMotorID, "Drive");
    tiltMotor.setPosition(0);
    spinMotor = new SparkFlex(GroundIntakeConstants.spinMotorID, MotorType.kBrushless);
  }

  @Override
  public void setAngle(Rotation2d target) {
    tiltMotor.setControl(new PositionVoltage(target.getRotations()).withEnableFOC(false));
  }

  @Override
  public void setSpeed(double speed) {
    spinMotor.set(speed);
  }

  @Override
  public void stopMotors() {
    spinMotor.stopMotor();
    tiltMotor.stopMotor();
  }

  @Override
  public void updateInputs(GroundIntakeIOInputs inputs) {}
}
