package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.lib.constants.RobotConstants;
import frc.lib.constants.RobotConstants.ElevatorConstants;
import frc.lib.util.BasePosition;
import org.littletonrobotics.junction.Logger;

public class ElevatorIONeo implements ElevatorIO {

  private final SparkMax leadMotor;
  private final SparkMax followMotor;

  private final RelativeEncoder encoder;

  private final SparkClosedLoopController controller;
  private final SparkMaxConfig leadConfig;
  private final SparkMaxConfig followConfig;

  private final DigitalInput bottomLimitSwitch;
  private final DigitalInput topLimitSwitch;

  private final double encoderUpperLimit = 0.0;
  private final double encoderLowerLimit = 0.0;
  // private final double rotationstoInches = 0.0;

  public ElevatorIONeo() {

    bottomLimitSwitch = new DigitalInput(RobotConstants.ElevatorConstants.bottomlimitswitchID);
    topLimitSwitch = new DigitalInput(RobotConstants.ElevatorConstants.toplimitswitchID);

    leadMotor = new SparkMax(RobotConstants.ElevatorConstants.leadMotorID, MotorType.kBrushless);
    followMotor =
        new SparkMax(RobotConstants.ElevatorConstants.followerMotorID, MotorType.kBrushless);
    followMotor.getEncoder().setPosition(0);

    encoder = leadMotor.getEncoder();
    encoder.setPosition(0);

    controller = leadMotor.getClosedLoopController();

    leadConfig = new SparkMaxConfig();
    leadConfig
        // .apply(new EncoderConfig().inverted(true))
        .apply(
        new ClosedLoopConfig()
            .pid(0.075, 0, 0)
            .minOutput(-0.5)
            .maxOutput(0.5)
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder));

    leadMotor.configure(leadConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    followConfig = new SparkMaxConfig();
    followConfig.follow(leadMotor, true);
    followConfig.apply(leadConfig);
    followMotor.configure(
        followConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setTargetPosition(BasePosition position) {
    double targetEncoderPosition = position.toRange(encoderLowerLimit, encoderUpperLimit);
    controller.setReference(targetEncoderPosition, ControlType.kPosition);
  }

  @Override
  public void moveToPoint(Rotation2d targetRot) {

    if (bottomLimitSwitch.get()) {
      encoder.setPosition(RobotConstants.ElevatorConstants.intakeheight.getRotations());
      // motor2
      //     .getEncoder()
      //     .setPosition(-RobotConstants.ElevatorConstants.intakeheight.getRotations());
    }
    if (topLimitSwitch.get()) {
      encoder.setPosition(RobotConstants.ElevatorConstants.L4height.getRotations());
      // motor2.getEncoder().setPosition(-RobotConstants.ElevatorConstants.L4height.getRotations());
    }

    controller.setReference(targetRot.getRotations(), ControlType.kPosition);
    // motor2.getClosedLoopController().setReference(-targetRot.getRotations(),
    // ControlType.kPosition);
  }

  @Override
  public void move(double input) {
    double realinput = input * 0.2;

    if (realinput > 0.15) {
      realinput = 0.15;
    }

    if (realinput < -0.15) {
      realinput = -0.15;
    }

    if (bottomLimitSwitch.get() && realinput < 0) {
      stopElevator();
    }
    if (topLimitSwitch.get() && realinput > 0) {
      stopElevator();
    }
    Logger.recordOutput("Elevator/encoder", encoder.getPosition());
    leadMotor.set(realinput);
  }

  @Override
  public RelativeEncoder getEncoder() {
    return encoder;
  }

  @Override
  public void stopElevator() {
    leadMotor.stopMotor();
  }

  @Override
  public void resetEncoder() {
    encoder.setPosition(0);
  }

  @Override
  public double getPercentRaised() {

    return (getEncoder().getPosition() / ElevatorConstants.maxHeight.getRotations());
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {}
}
