package frc.robot.subsystems.claw;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.RobotConstants;
import org.littletonrobotics.junction.Logger;

public class WristIONeo implements WristIO {
  private final SparkMax motor;
  private final SparkClosedLoopController pidController;
  private final RelativeEncoder encoder;
  private final PIDController controller = new PIDController(1, 0, 0);

  // Desired angle for the wrist
  private double desiredAngle = 0.0;

  // Constructor
  public WristIONeo() {
    SparkMaxConfig config = new SparkMaxConfig();

    config.closedLoop.maxMotion.maxAcceleration(300).maxVelocity(1000);

    config.closedLoop.p(1).i(0).d(0);

    config.alternateEncoder.positionConversionFactor(1);

    motor = new SparkMax(RobotConstants.EndEffector.wristmotorID, MotorType.kBrushless);
    pidController = motor.getClosedLoopController();
    encoder = motor.getAlternateEncoder();
    encoder.setPosition(0);
  }
  /// NEEDS TO BE CONFIGURED LOOP
  @Override
  public void setAngle(Rotation2d angle) {
    System.out.println("called");
    motor.set(angle.getDegrees());
    // pidController.setReference(angle.getRotations(),
    // SparkBase.ControlType.kMAXMotionPositionControl);
    Logger.recordOutput("wrist/angle", getAngle().getDegrees());
  }

  public Rotation2d getAngle() {

    return Rotation2d.fromRotations(
        encoder.getPosition()); // Adjust based on your encoder configuration
  }

  @Override
  public void stopMotor() {
    motor.stopMotor();
  }

  public void updateInputs(WristIOInputs inputs) {
    Logger.recordOutput("wrist/angle", getAngle().getDegrees());
    inputs.angle = getAngle().getDegrees();
  }
}
