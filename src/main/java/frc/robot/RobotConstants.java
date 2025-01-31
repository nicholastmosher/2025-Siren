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

  public static class Elevator {
    public static final int leadMotorID = 1;
    public static final int followerMotorID = 2;
    public static final int bottomlimitswitchID = 1;

    public static final double defaultheight = 0;
    public static final double intakeheight = 0;
    public static final double L1height = 0;
    public static final double L2height = 0;
    public static final double L3height = 0;
    public static final double L4height = 0;

    public static enum elevatorState {
      DEFAULT,
      INTAKE,
      L1,
      L2,
      L3,
      L4
    }
  }
}
