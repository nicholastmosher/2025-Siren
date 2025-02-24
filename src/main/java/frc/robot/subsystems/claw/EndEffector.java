// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.claw;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.constants.RobotConstants.EndEffectorConstants.WristState;
import org.littletonrobotics.junction.Logger;

public class EndEffector extends SubsystemBase {

  private final ClawIO claw;
  private final WristIO wrist;
  private WristState state;

  private final TrapezoidProfile profile;
  private final Timer timer;

  private WristIOInputsAutoLogged wristIOInputsAutoLogged;
  private ClawIOInputsAutoLogged clawIOInputsAutoLogged;

  /** Creates a new EndEffector. */
  public EndEffector(ClawIO clawimpl, WristIO wristimpl) {
    this.claw = clawimpl;
    this.wrist = wristimpl;
    this.state = WristState.DEFAULT;

    this.wristIOInputsAutoLogged = new WristIOInputsAutoLogged();
    this.clawIOInputsAutoLogged = new ClawIOInputsAutoLogged();

    profile = new TrapezoidProfile(new Constraints(0.5, 5));
    timer = new Timer();
    timer.start();
  }

  public void outEndEffector(double speed) {
    this.claw.setSpeed(speed);
  }

  public void inEndEffector(double speed) {
    this.claw.setSpeed(speed);
  }

  public void setWristAngle(WristState givenState) {
    this.state = givenState;
    timer.reset();
  }

  public void stopWrist() {
    this.wrist.stopMotor();
  }

  public void stopClaw() {
    this.claw.stopMotor();
  }

  public boolean getfrontIntaked() {
    return claw.getFrontIntaked();
  }

  public boolean getbackIntaked() {
    return claw.getIntakeIntaked();
  }

  @Override
  public void periodic() {
    Logger.recordOutput("wrist/state", this.state.toString());
    wrist.setAngle(
        Rotation2d.fromRotations(
            profile.calculate(
                    timer.getTimestamp(),
                    new State(this.wrist.getRotation(), this.wrist.getVelocity()),
                    new State(this.state.getTargetRotation2d().getRotations(), 0))
                .position));

    this.claw.updateInputs(this.clawIOInputsAutoLogged);
    this.wrist.updateInputs(this.wristIOInputsAutoLogged);
  }
}
