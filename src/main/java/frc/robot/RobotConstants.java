package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;

public class RobotConstants {

  public static class Intake {
    public static final int tiltMotorID = 1;
    public static final int spinMotorID = 2;

    public static final Rotation2d inactiveAngle = Rotation2d.fromDegrees(0);
    public static final Rotation2d activeAngle = Rotation2d.fromDegrees(0);
    public static final Rotation2d holdAngle = Rotation2d.fromDegrees(0);
  }

  public static class EndEffector {

    public static int wristmotorID = 1;

  }
}
