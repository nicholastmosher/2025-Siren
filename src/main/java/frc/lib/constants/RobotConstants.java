package frc.lib.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.lib.util.AllianceFlipUtil;

public class RobotConstants {

  public static class DriveConstants {

    public static final double alignP = 1.5;
    public static final double alignI = 0.0;
    public static final double alignD = 0.0;
    public static final double maxSpeed = 15.0;
    public static final double maxAccel = 150.0;
    public static final double translationRange = 0.005;

    public static final double headingP = 0.125 / 5.75;
    public static final double headingI = 0.01;
    public static final double headingD = 0.000;
    public static final double maxHeadingSpeed = 0.25;
    public static final double maxHeadingAccel = 1;
    public static final double headingRange = Units.degreesToRadians(1);
  }

  public static class IntakeConstants {
    public static final int tiltMotorID = 51;
    public static final int spinMotorID = 2;

    public static final Rotation2d inactiveAngle = Rotation2d.fromDegrees(0);
    public static final Rotation2d activeAngle = Rotation2d.fromDegrees(0);
    public static final Rotation2d holdAngle = Rotation2d.fromDegrees(0);
  }

  public static class ClimberConstants {

    public static final int climberMotorID = 50;
    public static final int limitSwitchID = 1;
  }

  public static class EndEffectorConstants {

    public static int wristmotorID = 32;
    public static int clawmotorID = 33;
    public static int frontcanrange = 40;
    public static int intakecanrange = 41;

    public static final Rotation2d defaultrot = new Rotation2d().fromRotations(0.7);
    public static final Rotation2d intakerot = new Rotation2d().fromRotations(0.815);
    public static final Rotation2d L1rot = new Rotation2d().fromRotations(0.72);
    public static final Rotation2d L2rot = new Rotation2d().fromRotations(0.72);
    public static final Rotation2d L3rot = new Rotation2d().fromRotations(0.72);
    public static final Rotation2d L4rot = new Rotation2d().fromRotations(0.72);

    public static final double staticSpeed = 0.0;
    public static final double intakeSpeed = -0.5;
    public static final double placeSpeed = -0.5;
    public static final double centerForwardSpeed = -0.15;
    public static final double centerBackwardsSpeed = 0.15;
  }

  public static class ElevatorConstants {
    public static final int leadMotorID = 30;
    public static final int followerMotorID = 31;
    public static final int bottomlimitswitchID = 4;
    public static final int toplimitswitchID = 2;

    public static final double closeEnoughPercent = 0.01;

    public static final Rotation2d defaultheight = Rotation2d.fromRotations(0);
    public static final Rotation2d intakeheight = Rotation2d.fromRotations(0);
    public static final Rotation2d L1height = Rotation2d.fromRotations(0);
    public static final Rotation2d L2height = Rotation2d.fromRotations(-18.7);
    public static final Rotation2d dealgifyheight = Rotation2d.fromRotations(-(17.5));
    public static final Rotation2d L3height = Rotation2d.fromRotations(-43);
    public static final Rotation2d L4height = Rotation2d.fromRotations(-70);
    public static final Rotation2d maxHeight = Rotation2d.fromRotations(72);
  }

  public static class GroundIntakeConstants {

    public static final int tiltMotorID = 51;
    public static final int spinMotorID = 34;

    public static final double staticSpeed = 0.0;
    public static final double intakeSpeed = 0.8;
    public static final double holdSpeed = 0.1;

    public static final Rotation2d defaultangle = Rotation2d.fromRotations(0);
    public static final Rotation2d intakingangle = Rotation2d.fromRotations(-4.5);
    public static final Rotation2d holdangle = Rotation2d.fromRotations(-2.5);
  }

  public static class GeneralConstants {

    public static boolean DEBUG = true;

    public static Pose2d[] reefPoses = {
      AllianceFlipUtil.apply(new Pose2d(new Translation2d(11.95, 3.90), Rotation2d.fromDegrees(0))),
      AllianceFlipUtil.apply(new Pose2d(new Translation2d(11.95, 4.24), Rotation2d.fromDegrees(0))),
      new Pose2d(),
      new Pose2d(),
      new Pose2d(),
      new Pose2d(),
      new Pose2d(),
      new Pose2d(),
      new Pose2d(),
      new Pose2d(),
      new Pose2d(),
      new Pose2d()
    };

    public static Pose2d[] intakePoses = {new Pose2d(), new Pose2d(), new Pose2d(), new Pose2d()};
  }
}
