// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PathToPose extends Command {
  // Drive drive;
  // Pose2d target;
  // PPHolonomicDriveController customPID;
  // PIDController customPIDX;
  // PIDController customPIDY;
  // ProfiledPIDController customPIDR;
  // HolonomicDriveController holonomicPID;
  // boolean usePathPlanner;
  // PathConstraints constraints;
  // List<Waypoint> waypoints;
  // Command pathplannerCommand;
  // boolean isBlue;
  /** Creates a new PathOnTheFlyToPose. */
  public PathToPose(Drive drive, Pose2d targetPose) {
    // this.drive = drive;
    // this.target = targetPose;

    // this.customPIDX = new PIDController(40, 0.1, 0.1);
    // this.customPIDY = new PIDController(40, 0.1, 0.1);
    // this.customPIDR = new ProfiledPIDController(75, 0.1, 4, new TrapezoidProfile.Constraints(0,
    // 0));
    // this.holonomicPID = new HolonomicDriveController(this.customPIDX, this.customPIDY,
    // this.customPIDR);
    // this.usePathPlanner = true;
    // this.constraints = PathConstraints.unlimitedConstraints(12);
    // this.pathplannerCommand = new InstantCommand();

    // boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Blue;
    // addRequirements(this.drive);
    // // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // if (Math.abs(drive.getPose().getX() - this.target.getX()) > 0.5
    //     && Math.abs(drive.getPose().getY() - this.target.getY()) > 0.5) {
    //   usePathPlanner = true;

    //   this.waypoints = List<Waypoint>(
    //     this.drive.getPose().interpolate(target, 0.5),
    //     this.target
    //   );

    //   PathPlannerPath path =
    //       new PathPlannerPath(
    //           this.waypoints, constraints, null, new GoalEndState(0, this.target.getRotation()));

    //   this.pathplannerCommand = AutoBuilder.followPath(path);

    //   this.pathplannerCommand.initialize();

    // } else {
    //   usePathPlanner = false;
    // }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (usePathPlanner) {
    //   this.pathplannerCommand.execute();
    // } else {
    //   // this.drive.runVelocity(this.holonomicPID.calculate(drive.getPose(), new
    // Trajectory.State(0.0, this.drive.get), this.target.getRotation()));

    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // this.pathplannerCommand.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return false;
    return false;
  }
}
