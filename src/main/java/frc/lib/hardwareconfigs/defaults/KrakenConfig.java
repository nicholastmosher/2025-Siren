// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.hardwareconfigs.defaults;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

/** Add your docs here. */
public class KrakenConfig {

  private TalonFXConfiguration config;

  public KrakenConfig() {

    config = new TalonFXConfiguration();
    config.CurrentLimits.withStatorCurrentLimit(40).withStatorCurrentLimitEnable(true);
    config.TorqueCurrent.withPeakForwardTorqueCurrent(40).withPeakReverseTorqueCurrent(40);
  }
}
