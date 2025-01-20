// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.groundintake;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.filter.Debouncer;
import frc.robot.subsystems.groundintake.IntakeIO.IntakeIOInputs;

/** Add your docs here. */
public class IntakeIOVortex implements IntakeIO {

  private final SparkFlex tiltMotor;
  private final SparkFlex spinMotor;

  private final SparkClosedLoopController tiltController;

  // Connection debouncers
  private final Debouncer tiltConnectedDebounce = new Debouncer(0.5);
  private final Debouncer spinConnectedDebounce = new Debouncer(0.5);

  public IntakeIOVortex() {
    tiltMotor =
        new SparkFlex(IntakeConstants.tiltMotorID, MotorType.fromId(IntakeConstants.tiltMotorID));
    spinMotor =
        new SparkFlex(IntakeConstants.spinMotorID, MotorType.fromId(IntakeConstants.spinMotorID));

    tiltController = tiltMotor.getClosedLoopController();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // Refresh all signals
    var tiltMotorStatus = true;
    var spinMotorStatus =
        true; // BaseStatusSignal.refreshAll(turnPosition, turnVelocity, turnAppliedVolts,
    // turnCurrent);

    inputs.tiltMotorConnected = tiltConnectedDebounce.calculate(tiltMotorStatus);
  }
}
