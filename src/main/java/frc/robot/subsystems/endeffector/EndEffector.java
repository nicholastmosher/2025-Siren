// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endeffector;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.enums.robotstate;
import frc.robot.subsystems.statehandler.StateHandler;

public class EndEffector extends SubsystemBase {

  private final ClawIO claw;
  private final WristIO wrist;
  private final StateHandler stateHandler;

  private final TrapezoidProfile profile;
  private final Timer timer;

  private WristIOInputsAutoLogged wristIOInputsAutoLogged;
  private ClawIOInputsAutoLogged clawIOInputsAutoLogged;

  /** Creates a new EndEffector. */
  public EndEffector(ClawIO clawimpl, WristIO wristimpl, StateHandler handler) {
    this.claw = clawimpl;
    this.wrist = wristimpl;
    this.stateHandler = handler;

    this.wristIOInputsAutoLogged = new WristIOInputsAutoLogged();
    this.clawIOInputsAutoLogged = new ClawIOInputsAutoLogged();

    profile = new TrapezoidProfile(new Constraints(0.5, 5));
    timer = new Timer();
    timer.start();
  }

  public void stopWrist() {
    this.wrist.stopMotor();
    this.timer.stop();
  }

  public boolean getfrontIntaked() {
    return claw.getFrontIntaked();
  }

  public boolean getbackIntaked() {
    return claw.getIntakeIntaked();
  }

  @Override
  public void periodic() {

    if (this.stateHandler.getState() == robotstate.STOP) {
      this.claw.stopMotor();
      stopWrist();
    } else {
      timer.start();
    }

    if (timer.isRunning()) {
      wrist.setAngle(
          Rotation2d.fromRotations(
              profile.calculate(
                      timer.getTimestamp(),
                      new State(this.wrist.getRotation(), this.wrist.getVelocity()),
                      new State(this.stateHandler.getState().getWristTarget().getRotations(), 0))
                  .position));
    }
  }
}
