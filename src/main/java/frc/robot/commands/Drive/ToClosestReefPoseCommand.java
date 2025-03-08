package frc.robot.commands.Drive;

import static frc.lib.constants.RobotConstants.GeneralConstants.reefPoses;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.util.GeometryUtil;
import frc.robot.subsystems.drive.Drive;

public class ToClosestReefPoseCommand extends Command {
  private final Drive drive;
  private Command driveToPose;

  public ToClosestReefPoseCommand(Drive drive) {
    this.drive = drive;
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.drive);

    driveToPose = new InstantCommand();
  }

  @Override
  public void initialize() {
    Pose2d closestpose = new Pose2d();
    double closestDistance = 900000000;
    for (int i = 0; i < reefPoses.length; i++) {
      double distance =
          GeometryUtil.toTransform2d(drive.getPose())
              .getTranslation()
              .getDistance(GeometryUtil.toTransform2d(reefPoses[i]).getTranslation());
      if (distance < closestDistance) {
        closestpose =
            new Pose2d(new Translation2d(11.9, 4.2), Rotation2d.fromDegrees(0)); // intakePoses[i];
      }
    }

    driveToPose = new AlignToPoseCommand(this.drive, closestpose);
    driveToPose.initialize();
  }

  @Override
  public void execute() {
    driveToPose.execute();
  }

  @Override
  public boolean isFinished() {
    // TODO: Make this return true when this Command no longer needs to run execute()
    return driveToPose.isFinished();
  }

  @Override
  public void end(boolean interrupted) {
    driveToPose.end(interrupted);
  }
}
