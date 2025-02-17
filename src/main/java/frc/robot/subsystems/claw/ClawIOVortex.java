package frc.robot.subsystems.claw;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ClawIOVortex implements ClawIO {

  private final SparkFlex motor;
  private final SparkClosedLoopController m_controller;

  // Current wheel speed (for logging or feedback)
  private double currentSpeed = 0.0;

  // Constructor
  public ClawIOVortex(int motorPort) {
    motor = new SparkFlex(motorPort, MotorType.kBrushless);

    SparkMaxConfig config = new SparkMaxConfig();
    config.closedLoop.p(10).i(0).d(0).velocityFF(1);
    m_controller = motor.getClosedLoopController();
  }

  @Override
  public void setSpeed(double speed) {
    motor.set(speed); // setReference(speed, ControlType.kMAXMotionVelocityControl);
  }

  @Override
  public double getCurrentSpeed() {
    return motor.getEncoder().getVelocity();
  }

  @Override
  public double getPosition() {
    return motor.getEncoder().getPosition();
  }

  @Override
  public void stopMotor() {
    motor.stopMotor();
  }
}
