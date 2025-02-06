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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.RobotConstants;
import frc.robot.RobotConstants.Elevator.elevatorState;

public class ElevatorIONeo implements ElevatorIO {

  private final SparkMax leadMotor;
  private final SparkMax followerMotor;

  private final AbsoluteEncoder encoder;

  private final SparkClosedLoopController pid;

  private final SparkMaxConfig leadConfig;
  private final SparkMaxConfig followerConfig;

  private final DigitalInput limitswitchBottom;

  private final DigitalInput bottomLimitSwitch;
  private final DigitalInput topLimitSwitch;

  public ElevatorIONeo() {

    bottomLimitSwitch = new DigitalInput(RobotConstants.Elevator.bottomlimitswitchID);
    topLimitSwitch = new DigitalInput(RobotConstants.Elevator.toplimitswitchID);

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
  public void moveToState(RobotConstants.Elevator.elevatorState state) {
    boolean atLowestPoint = false;
    boolean atHighestPoint = false;
    if (bottomLimitSwitch.get()) {
      atLowestPoint = true;
    }
    if (topLimitSwitch.get()) {
      atHighestPoint = true;
    }
    switch (state) {
      case DEFAULT:
        moveToPoint(RobotConstants.Elevator.defaultheight);
        break;

      case INTAKE:
        moveToPoint(RobotConstants.Elevator.intakeheight);
        break;

      case L1:
        moveToPoint(RobotConstants.Elevator.L1height);
        break;

      case L2:
        moveToPoint(RobotConstants.Elevator.L2height);
        break;

      case L3:
        moveToPoint(RobotConstants.Elevator.L3height);
        break;

      case L4:
        moveToPoint(RobotConstants.Elevator.L4height);
        break;
    }
  }

  public void moveToPoint(Rotation2d targetRot) {
    if (bottomLimitSwitch.get() && targetRot.getDegrees() < Rotation2d.fromRotations(encoder.getPosition()).getDegrees()) {
      stopElevator();
    }
    if (topLimitSwitch.get() && targetRot.getDegrees() > Rotation2d.fromRotations(encoder.getPosition()).getDegrees()) {
      stopElevator();
    }
    pid.setReference(targetRot.getDegrees(), ControlType.kMAXMotionPositionControl);
  }

  @Override
  public void stopElevator() {
    leadMotor.stopMotor();
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {}
}
