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
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.RobotConstants;

public class ElevatorIONeo implements ElevatorIO {

  private final SparkMax leadMotor;
  private final SparkMax followerMotor;

  private final AbsoluteEncoder encoder;

  private final SparkClosedLoopController pid;

  private final SparkMaxConfig leadConfig;
  private final SparkMaxConfig followerConfig;

  private final DigitalInput limitswitchBottom;

  public ElevatorIONeo() {

    leadMotor = new SparkMax(RobotConstants.Elevator.leadMotorID, MotorType.kBrushless);
    followerMotor = new SparkMax(RobotConstants.Elevator.leadMotorID, MotorType.kBrushless);
    limitswitchBottom = new DigitalInput(RobotConstants.Elevator.bottomlimitswitchID);

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

  @Override
  public void moveToPoint(RobotConstants.Elevator.elevatorState state) {
    switch (state) {
      case DEFAULT:
        pid.setReference(
            RobotConstants.Elevator.defaultheight, ControlType.kMAXMotionPositionControl);
        break;

      case INTAKE:
        pid.setReference(
            RobotConstants.Elevator.intakeheight, ControlType.kMAXMotionPositionControl);
        break;

      case L1:
        pid.setReference(RobotConstants.Elevator.L1height, ControlType.kMAXMotionPositionControl);
        break;

      case L2:
        pid.setReference(RobotConstants.Elevator.L2height, ControlType.kMAXMotionPositionControl);
        break;

      case L3:
        pid.setReference(RobotConstants.Elevator.L3height, ControlType.kMAXMotionPositionControl);
        break;

      case L4:
        pid.setReference(RobotConstants.Elevator.L4height, ControlType.kMAXMotionPositionControl);
        break;
    }
  }

  @Override
  public void stopElevator() {}

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {}
}
