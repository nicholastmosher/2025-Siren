package frc.robot.subsystems.climber;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.constants.RobotConstants;

public class ClimberIOKraken implements ClimberIO {
  private final TalonFX motor;

  // Constructor
  public ClimberIOKraken() {
    motor = new TalonFX(RobotConstants.ClimberConstants.climberMotorID, "Drive");
  }

  @Override
  public void setAngle(Rotation2d angle) {
    motor.setPosition(angle.getDegrees());
  }

  public double getAngle() {

    return motor.getPosition().getValueAsDouble();
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {}

  @Override
  public void stopMotor() {
    motor.stopMotor();
  }
}
