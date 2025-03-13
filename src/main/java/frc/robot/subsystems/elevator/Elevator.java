// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.constants.RobotConstants.ElevatorConstants;
import frc.lib.util.BasePosition;
import frc.robot.subsystems.virtualsubsystems.statehandler.StateHandler;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  public static final BasePosition CORAL_L1 = new BasePosition(0.0);
  public static final BasePosition CORAL_L2 = new BasePosition(0.25);
  public static final BasePosition CORAL_L3 = new BasePosition(0.50);
  public static final BasePosition CORAL_L4 = new BasePosition(0.95);
  public static final BasePosition BOTTOM = new BasePosition(0.0);
  private final double encoderLowerLimit = 0.0;
  private final double encoderUpperLimit = 280.0;

  /** Creates a new Elevator. */
  private final ElevatorIO elevator;

  private ElevatorIOInputsAutoLogged elevatorinputs;
  private final StateHandler stateHandler;

  private TrapezoidProfile profile;
  private ElevatorFeedforward feedforward;
  private Timer profileTimer;

  private double targetRotation;

  private BasePosition ElevatorPositionNormalized;

  public Elevator(ElevatorIO elevator, StateHandler handler) {
    this.elevator = elevator;
    elevatorinputs = new ElevatorIOInputsAutoLogged();
    this.stateHandler = handler;

    profile = new TrapezoidProfile(new Constraints(200, 2000));
    feedforward = new ElevatorFeedforward(0, 0.0, 0);
    profileTimer = new Timer();
    profileTimer.start();

    targetRotation = this.stateHandler.getState().getElevatorHeight().getRotations();
    ElevatorPositionNormalized = new BasePosition(0.0);
  }

  public void setTargetPosition(BasePosition position) {
    ElevatorPositionNormalized = position;
  }

  public BasePosition getBasePosition() {
    return elevator.getBasePosition();
  }

  public void moveElevator(double input) {
    this.elevator.move(input);
  }

  public void stopElevator() {
    elevator.stopElevator();
    profileTimer.stop();
  }

  public void resetEncoder() {
    this.elevator.resetEncoder();
  }

  public double getPercentRaised() {
    return elevator.getPercentRaised();
  }

  public boolean isCloseEnough() {

    double encoderposition = elevator.getEncoder().getPosition();
    double targetposition = this.stateHandler.getState().getElevatorHeight().getRotations();

    if (targetposition < (encoderposition * (1 + ElevatorConstants.closeEnoughPercent))
        && targetposition > (encoderposition * (1 + ElevatorConstants.closeEnoughPercent))) {
      return true;
    }

    return false;
  }

  @Override
  public void periodic() {
    elevator.periodic();

    if (this.targetRotation != this.stateHandler.getState().getElevatorHeight().getRotations()) {
      profileTimer.reset();
      this.targetRotation = this.stateHandler.getState().getElevatorHeight().getRotations();
    }

    if (stateHandler.getState().isDisabled()) {
      profileTimer.stop();
      elevator.stopElevator();
    } else {
      profileTimer.start();
    }
    if (profileTimer.isRunning()) {

      Rotation2d targetRotation =
          Rotation2d.fromRotations(
              profile.calculate(
                      profileTimer.getTimestamp(),
                      new State(
                          this.elevator.getEncoder().getPosition(),
                          this.elevator.getEncoder().getVelocity()),
                      new State(
                          ElevatorPositionNormalized.toRange(encoderLowerLimit, encoderUpperLimit),
                          0))
                  .position);

      elevator.setTargetPosition(
          BasePosition.fromRange(
              encoderLowerLimit, encoderUpperLimit, targetRotation.getRotations()));
    }

    elevator.updateInputs(elevatorinputs);
    Logger.processInputs("Elevator", elevatorinputs);
  }
}
