// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.AllianceFlipUtil;
import frc.robot.subsystems.drive.Drive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToPose extends Command {
  Drive drive;
  Pose2d target;

  ProfiledPIDController xController;
  ProfiledPIDController yController;
  ProfiledPIDController rotController;

  boolean isBlue;

  /** Creates a new DriveToPose. */
  public DriveToPose(Drive drive, Pose2d pose) {
    this.drive = drive;
    addRequirements(drive);

    this.target = pose;

    xController = new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(1, 1));
    yController = new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(1, 1));
    rotController = new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(1, 1));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.isBlue = DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Blue;

    if (!this.isBlue) {
      this.target = AllianceFlipUtil.apply(this.target);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentPose = drive.getPose();
    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;

    ChassisSpeeds speeds =
        new ChassisSpeeds(
            xController.calculate(currentPose.getX(), target.getX()),
            yController.calculate(currentPose.getY(), target.getY()),
            rotController.calculate(
                currentPose.getRotation().getDegrees(), target.getRotation().getDegrees()));

    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds,
            false ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
