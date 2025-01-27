package frc.robot.subsystems.Elevator;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.RobotConstants;

public class ElevatorIONeo implements ElevatorIO {

  private final SparkMax leadMotor;
  private final SparkMax followerMotor;

  private final AbsoluteEncoder encoder;

  private final SparkClosedLoopController pid;

  private final SparkMaxConfig leadConfig;
  private final SparkMaxConfig followerConfig;

  public ElevatorIONeo() {

    leadMotor = new SparkMax(RobotConstants.Elevator.leadMotorID, MotorType.kBrushless);
    followerMotor = new SparkMax(RobotConstants.Elevator.leadMotorID, MotorType.kBrushless);
    encoder = leadMotor.getAbsoluteEncoder();
    pid = leadMotor.getClosedLoopController();

    leadConfig = new SparkMaxConfig();
    leadConfig
        .closedLoop
        .pid(0, 0, 0)
        .iZone(0)
        .minOutput(0)
        .maxOutput(0)
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .maxMotion
        .allowedClosedLoopError(0)
        .maxAcceleration(0)
        .maxVelocity(0);

    leadMotor.configure(leadConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    followerConfig = new SparkMaxConfig();
    followerConfig.apply(leadConfig);
    followerConfig.follow(RobotConstants.Elevator.leadMotorID);
    followerMotor.configure(
        followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void moveToPoint(double value) {
    pid.setReference(value, ControlType.kMAXMotionPositionControl);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {}
}
