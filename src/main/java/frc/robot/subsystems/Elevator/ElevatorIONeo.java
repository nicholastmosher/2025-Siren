package frc.robot.subsystems.Elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.RobotConstants;
import frc.robot.RobotConstants.ElevatorConstants.elevatorState;
import org.littletonrobotics.junction.Logger;

public class ElevatorIONeo implements ElevatorIO {

  private final SparkMax leadMotor;
  private final SparkMax motor2;

  private final RelativeEncoder encoder;

  private final SparkClosedLoopController leadpid;
  private final SparkClosedLoopController pid2;

  private final SparkMaxConfig leadConfig;
  private final SparkMaxConfig motor2config;

  // private final DigitalInput limitswitchBottom;

  // private final DigitalInput bottomLimitSwitch;
  // private final DigitalInput topLimitSwitch;

  private elevatorState state = elevatorState.DEFAULT;

  public ElevatorIONeo() {

    // bottomLimitSwitch = new DigitalInput(RobotConstants.Elevator.bottomlimitswitchID);
    // topLimitSwitch = new DigitalInput(RobotConstants.Elevator.toplimitswitchID);

    leadMotor = new SparkMax(RobotConstants.ElevatorConstants.leadMotorID, MotorType.kBrushless);
    motor2 = new SparkMax(RobotConstants.ElevatorConstants.followerMotorID, MotorType.kBrushless);
    // limitswitchBottom = new DigitalInput(RobotConstants.Elevator.bottomlimitswitchID);

    encoder = leadMotor.getEncoder();
    encoder.setPosition(0);
    leadpid = leadMotor.getClosedLoopController();
    pid2 = motor2.getClosedLoopController();

    leadConfig = new SparkMaxConfig();

    leadConfig
        .closedLoop
        .pid(0.0025, 0, 0)
        .minOutput(-1)
        .maxOutput(1)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .maxMotion
        .allowedClosedLoopError(0.3)
        .maxAcceleration(56000)
        .maxVelocity(5600);

    leadMotor.configure(leadConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    motor2config = new SparkMaxConfig();
    motor2config.apply(leadConfig);
    motor2config.follow(leadMotor, true);
    motor2.configure(motor2config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void moveToState(RobotConstants.ElevatorConstants.elevatorState state) {
    boolean atLowestPoint = false;
    boolean atHighestPoint = false;
    // if (bottomLimitSwitch.get()) {
    //   atLowestPoint = true;
    // }
    // if (topLimitSwitch.get()) {
    //   atHighestPoint = true;
    // }
    this.state = state;
    moveToPoint(state.getTargetRotation2d());
  }

  @Override
  public void moveToPoint(Rotation2d targetRot) {
    // if (bottomLimitSwitch.get()
    //     && targetRot.getDegrees() < Rotation2d.fromRotations(encoder.getPosition()).getDegrees())
    // {
    //   stopElevator();
    // }
    // if (topLimitSwitch.get()
    //     && targetRot.getDegrees() > Rotation2d.fromRotations(encoder.getPosition()).getDegrees())
    // {
    //   stopElevator();
    // }
    leadpid.setReference(targetRot.getRotations(), ControlType.kMAXMotionPositionControl);
    // pid2.setReference((-getEncoder()), ControlType.kMAXMotionPositionControl);
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

    leadMotor.set(realinput);
  }

  @Override
  public double getEncoder() {
    return encoder.getPosition();
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
  public void updateInputs(ElevatorIOInputs inputs) {
    Logger.recordOutput("elevator/encoder", getEncoder());
    inputs.enumState = state.name();
  }
}
