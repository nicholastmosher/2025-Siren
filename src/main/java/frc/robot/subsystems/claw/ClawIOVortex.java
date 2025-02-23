package frc.robot.subsystems.claw;

import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.hardware.CANrange;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.RobotConstants;
import org.littletonrobotics.junction.Logger;

public class ClawIOVortex implements ClawIO {

  private final SparkFlex motor;
  private final SparkClosedLoopController m_controller;

  private final CANrange frontCaNrange;
  private final CANrange intakeCaNrange;

  // Constructor
  public ClawIOVortex() {
    motor = new SparkFlex(RobotConstants.EndEffectorConstants.clawmotorID, MotorType.kBrushless);

    SparkMaxConfig config = new SparkMaxConfig();
    config
        .closedLoop
        .pid(0.0025, 0, 2)
        .minOutput(-1)
        .maxOutput(1)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .maxMotion
        .allowedClosedLoopError(0.5);

    m_controller = motor.getClosedLoopController();

    frontCaNrange = new CANrange(RobotConstants.EndEffectorConstants.frontcanrange);
    frontCaNrange
        .getConfigurator()
        .apply(new ProximityParamsConfigs().withProximityThreshold(0.15));

    intakeCaNrange = new CANrange(RobotConstants.EndEffectorConstants.frontcanrange);
    intakeCaNrange
        .getConfigurator()
        .apply(new ProximityParamsConfigs().withProximityThreshold(0.15));
  }

  @Override
  public void setSpeed(double speed) {
    motor.set(speed);
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
  public boolean getFrontIntaked() {
    Logger.recordOutput("claw/canrangefront", frontCaNrange.getDistance().getValueAsDouble());
    Logger.recordOutput("claw/canrangefrontbool", frontCaNrange.getIsDetected().getValue());

    return frontCaNrange.getIsDetected().getValue();
  }

  @Override
  public boolean getIntakeIntaked() {
    Logger.recordOutput("claw/canrangefront", intakeCaNrange.getDistance().getValueAsDouble());
    Logger.recordOutput("claw/canrangefrontbool", intakeCaNrange.getIsDetected().getValue());

    return intakeCaNrange.getIsDetected().getValue();
  }

  @Override
  public void stopMotor() {
    motor.stopMotor();
  }
}
