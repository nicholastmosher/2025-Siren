package frc.robot.commands.Drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.constants.RobotConstants;
import frc.lib.util.AllianceFlipUtil;
import frc.lib.util.LoggedTunableNumber;
import frc.robot.subsystems.drive.Drive;

public class AlignToPoseCommand extends Command {
  private final Drive drive;
  private Pose2d target;

  private final LoggedTunableNumber translationp;
  private final LoggedTunableNumber translationi;
  private final LoggedTunableNumber translationd;

  private final LoggedTunableNumber headingp;
  private final LoggedTunableNumber headingi;
  private final LoggedTunableNumber headingd;

  private final ProfiledPIDController alignXController;
  private final ProfiledPIDController alignYController;
  private final ProfiledPIDController alignHeadingController;

  public AlignToPoseCommand(Drive drive, Pose2d targetPose) {
    this.drive = drive;
    this.target = targetPose;
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.drive);

    translationp = new LoggedTunableNumber("translationp", RobotConstants.DriveConstants.alignP);
    translationi = new LoggedTunableNumber("translationi", RobotConstants.DriveConstants.alignI);
    translationd = new LoggedTunableNumber("translationd", RobotConstants.DriveConstants.alignD);

    headingp = new LoggedTunableNumber("headingp", RobotConstants.DriveConstants.headingP);
    headingi = new LoggedTunableNumber("headingi", RobotConstants.DriveConstants.headingI);
    headingd = new LoggedTunableNumber("headingd", RobotConstants.DriveConstants.headingD);

    alignXController =
        new ProfiledPIDController(
            RobotConstants.DriveConstants.alignP,
            RobotConstants.DriveConstants.alignI,
            RobotConstants.DriveConstants.alignD,
            new TrapezoidProfile.Constraints(
                RobotConstants.DriveConstants.maxSpeed, RobotConstants.DriveConstants.maxAccel));
    alignXController.setTolerance(RobotConstants.DriveConstants.translationRange);

    alignYController =
        new ProfiledPIDController(
            RobotConstants.DriveConstants.alignP,
            RobotConstants.DriveConstants.alignI,
            RobotConstants.DriveConstants.alignD,
            new TrapezoidProfile.Constraints(
                RobotConstants.DriveConstants.maxSpeed, RobotConstants.DriveConstants.maxAccel));
    alignYController.setTolerance(RobotConstants.DriveConstants.translationRange);

    alignHeadingController =
        new ProfiledPIDController(
            RobotConstants.DriveConstants.headingP,
            RobotConstants.DriveConstants.headingI,
            RobotConstants.DriveConstants.headingD,
            new TrapezoidProfile.Constraints(
                RobotConstants.DriveConstants.maxHeadingSpeed,
                RobotConstants.DriveConstants.maxHeadingAccel));
    alignHeadingController.enableContinuousInput(-Math.PI, Math.PI);
    alignHeadingController.setTolerance(Units.degreesToRadians(1.0));
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
      target = AllianceFlipUtil.apply(target);
    }
  }

  @Override
  public void initialize() {
    setTranslationPIDConstants(
        translationp.getAsDouble(), translationi.getAsDouble(), translationd.getAsDouble());
    setHeadingPIDConstants(headingp.getAsDouble(), headingi.getAsDouble(), headingd.getAsDouble());
  }

  @Override
  public void execute() {
    this.drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            DriveCommands.driveFieldOriented(
                this.drive,
                this.alignXController.calculate(this.drive.getPose().getX(), this.target.getX()),
                this.alignYController.calculate(this.drive.getPose().getY(), this.target.getY()),
                this.alignHeadingController.calculate(
                    this.drive.getPose().getRotation().getRadians(),
                    this.target.getRotation().getRadians())),
            false ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation()));
  }

  @Override
  public boolean isFinished() {
    return (isAligned());
  }

  @Override
  public void end(boolean interrupted) {
    this.drive.stop();
    ;
  }

  public final void setTranslationPIDConstants(double kp, double ki, double kd) {
    alignXController.setPID(kp, ki, kd);
    alignYController.setPID(kp, ki, kd);
  }

  public final void setHeadingPIDConstants(double kp, double ki, double kd) {
    alignHeadingController.setPID(kp, ki, kd);
  }

  public final double calculateTranslationX(double currentpose, double setpoint) {
    return alignXController.calculate(currentpose, setpoint);
  }

  public final double calculateTranslationy(double currentpose, double setpoint) {
    return alignYController.calculate(currentpose, setpoint);
  }

  public final double calculateHeading(double currentpose, double setpoint) {
    return alignHeadingController.calculate(currentpose, setpoint);
  }

  public final boolean isAligned() {
    return (alignXController.atGoal()
        && alignYController.atGoal()
        && alignHeadingController.atGoal());
  }
}
