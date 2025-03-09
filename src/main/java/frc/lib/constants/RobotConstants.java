package frc.lib.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class RobotConstants {

  public static class DriveConstants {

    public static final double alignP = 0.8;
    public static final double alignI = 0.25;
    public static final double alignD = 0.1;
    public static final double maxSpeed = 15.0;
    public static final double maxAccel = 150.0;
    public static final double translationRange = 0.05;

    public static final double headingP = 0.0; // 0.125 / 22;
    public static final double headingI = 0.00000;
    public static final double headingD = 0.000;
    public static final double maxHeadingSpeed = 2;
    public static final double maxHeadingAccel = 100;
    public static final double headingRange = Units.degreesToRadians(160);
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
    public static final Rotation2d intakerot = new Rotation2d().fromRotations(0.8);
    public static final Rotation2d L1rot = new Rotation2d().fromRotations(0.72);
    public static final Rotation2d L2rot = new Rotation2d().fromRotations(0.72);
    public static final Rotation2d L3rot = new Rotation2d().fromRotations(0.73);
    public static final Rotation2d L4rot = new Rotation2d().fromRotations(0.7);

    public static final double staticSpeed = 0.0;
    public static final double intakeSpeed = -1;
    public static final double placeSpeed = -1;
    public static final double centerForwardSpeed = -0.3;
    public static final double centerBackwardsSpeed = 0.3;
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
    public static final Rotation2d L3height = Rotation2d.fromRotations(-35.4);
    public static final Rotation2d L4height = Rotation2d.fromRotations(-70);
    public static final Rotation2d maxHeight = Rotation2d.fromRotations(72);
  }

  public static class GroundIntakeConstants {

    public static final int tiltMotorID = 51;
    public static final int spinMotorID = 34;

    public static final double staticSpeed = 0.0;
    public static final double intakeSpeed = 1;
    public static final double throwSpeed = -1;
    public static final double holdSpeed = 0.1;

    public static final Rotation2d defaultangle = Rotation2d.fromRotations(0);
    public static final Rotation2d intakingangle = Rotation2d.fromRotations(-9.5);
    public static final Rotation2d holdangle = Rotation2d.fromRotations(0);
  }

  public static class GeneralConstants {

    public static Pose2d[] getCartesianCoordinates(
        double angle, double poseOffset, double poseOffsetBack) { // Left:trueRight:false
      // Calculate x and y using trigonometric functions
      double x = (0.8315 + poseOffsetBack) * Math.cos(angle) + 4.4895;
      double y = (0.8315 + poseOffsetBack) * Math.sin(angle) + 4.026;

      // Compute the tangent line's direction
      double tangentX = -Math.sin(angle); // Negative sine for perpendicular direction
      double tangentY = Math.cos(angle); // Cosine for perpendicular direction

      // Normalize the tangent direction
      double magnitude = Math.sqrt(tangentX * tangentX + tangentY * tangentY);
      tangentX /= magnitude;
      tangentY /= magnitude;

      // Calculate pose1 and pose2 positions along the tangent line
      double x1 = x + tangentX * poseOffset;
      double y1 = y + tangentY * poseOffset;

      double x2 = x - tangentX * poseOffset;
      double y2 = y - tangentY * poseOffset;

      // Create Pose2d objects
      Pose2d pose1 = new Pose2d(new Translation2d(x1, y1), Rotation2d.fromRadians(angle));
      Pose2d pose2 = new Pose2d(new Translation2d(x2, y2), Rotation2d.fromRadians(angle));

      // Return the coordinates as a Pose2d array
      return new Pose2d[] {pose1, pose2};
    }

    public static boolean DEBUG = true;

    public static Pose2d[] farSide = getCartesianCoordinates(0, 0.165, 0.44);

    public static Pose2d[] reefPoses = {
      farSide[1],
      farSide[0],
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
