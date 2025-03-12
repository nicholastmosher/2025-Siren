// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.commands.Drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

public class DriveCommands {
  private static final double DEADBAND = 0.1;

  private DriveCommands() {}

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Get linear velocity
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          // Apply rotation deadband
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square rotation value for more precise control
          omega = Math.copySign(omega * omega, omega);

          // Convert to field relative speeds & send command
          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec());
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds,
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        },
        drive);
  }

  public static ChassisSpeeds driveFieldOriented(
      Drive drive, Double xSupplier, Double ySupplier, Double omegaSupplier) {
    Translation2d linearVelocity = getLinearVelocityFromJoysticks(xSupplier, ySupplier);

    // Apply rotation deadband
    double omega = MathUtil.applyDeadband(omegaSupplier, 0);

    // Square rotation value for more precise control

    // Convert to field relative speeds & send command
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
            linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
            omega * drive.getMaxAngularSpeedRadPerSec());

    return speeds;
  }

  /**
   * Drive to the given target pose.
   * 
   * If the robot is far, it will use PathPlanner with dynamic waypoints.
   * If the robot is near, it will use a custom PID controller.
   */
  public static Command driveToPose(Drive drive, Pose2d target) {
    var deltaX = Math.abs(drive.getPose().getX() - target.getX());
    var deltaY = Math.abs(drive.getPose().getY() - target.getY());

    if (deltaX > 0.5 && deltaY > 0.5) {
      return driveToPosePathPlanner(drive, target);
    } else {
      return driveToPoseCustomPid(drive, target);
    }
  }

  public static Command driveToPosePathPlanner(Drive drive, Pose2d target) {
    var startPose = new Pose2d(
      new Translation2d(5.38, 5.39),
      Rotation2d.fromDegrees(62)
    );
    // Maybe use this for start pose instead?
    // var startPose = drive.getPose();
    var endPose = target;
    var waypoints = PathPlannerPath.waypointsFromPoses(startPose, endPose);
    var constraints = PathConstraints.unlimitedConstraints(12);
    var path = new PathPlannerPath(
      waypoints,
      constraints,
      null,
      new GoalEndState(0, target.getRotation())
    );

    boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Blue;
    if (!isBlue) {
      path = path.flipPath();
    }

    return AutoBuilder.followPath(path);
  }

  public static Command driveToPoseCustomPid(Drive drive, Pose2d target) {
    var xPid = new PIDController(40, 0.1, 0.1);
    xPid.setTolerance(0.01);

    var yPid = new PIDController(40, 0.1, 0.1);
    yPid.setTolerance(0.01);

    var rotPid = new PIDController(75, 0, 0);
    rotPid.setTolerance(1);
    rotPid.enableContinuousInput(-180, 180);

    return driveToPoseCustomPid(
      drive,
      target,
      xPid,
      yPid,
      rotPid
    );
  }

  public static Command driveToPoseCustomPid(
    Drive drive, 
    Pose2d target,
    PIDController xPid,
    PIDController yPid,
    PIDController rotPid
  ) {
    var drivePose = drive.getPose();
    var xOutput = xPid.calculate(drivePose.getX(), target.getX());
    var yOutput = yPid.calculate(drivePose.getY(), target.getY());
    var rotOutput = rotPid.calculate(
      drivePose.getRotation().getRadians(),
      target.getRotation().getRadians()
    );
    var speeds = new ChassisSpeeds(xOutput, yOutput, rotOutput);

    return Commands.run(() -> drive.runVelocity(speeds));
  }
}
