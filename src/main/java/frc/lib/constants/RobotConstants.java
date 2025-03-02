package frc.lib.constants;

import edu.wpi.first.math.geometry.Rotation2d;

public class RobotConstants {

  public static class IntakeConstants {
    public static final int tiltMotorID = 1;
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
    public static final Rotation2d L1rot = new Rotation2d().fromRotations(0.7);
    public static final Rotation2d L2rot = new Rotation2d().fromRotations(0.7);
    public static final Rotation2d L3rot = new Rotation2d().fromRotations(0.7);
    public static final Rotation2d L4rot = new Rotation2d().fromRotations(0.7);

    public static final double staticSpeed = 0.0;
    public static final double intakeSpeed = 0.0;
    public static final double centerForwardSpeed = 0.0;
    public static final double centerBackwardsSpeed = 0.0;
  }

  public static class ElevatorConstants {
    public static final int leadMotorID = 30;
    public static final int followerMotorID = 31;
    public static final int bottomlimitswitchID = 1;
    public static final int toplimitswitchID = 1;

    public static final double closeEnoughPercent = 0.01;

    public static final Rotation2d defaultheight = new Rotation2d().fromRotations(0);
    public static final Rotation2d intakeheight = new Rotation2d().fromRotations(0);
    public static final Rotation2d L1height = new Rotation2d().fromRotations(0);
    public static final Rotation2d L2height = new Rotation2d().fromRotations(-15);
    public static final Rotation2d dealgifyheight = new Rotation2d().fromRotations(-(17.5));
    public static final Rotation2d L3height = new Rotation2d().fromRotations(-37.5);
    public static final Rotation2d L4height = new Rotation2d().fromRotations(-70);
    public static final Rotation2d maxHeight = new Rotation2d().fromRotations(72);
  }

  public static class GroundIntakeConstants {

    public static final int tiltMotorID = 1;
    public static final int spinMotorID = 1;

    public static final double staticSpeed = 0.0;
    public static final double intakeSpeed = 0.0;
    public static final double holdSpeed = 0.0;

    public static final Rotation2d defaultangle = Rotation2d.fromRotations(0);
    public static final Rotation2d intakingangle = Rotation2d.fromRotations(0);
    public static final Rotation2d holdangle = Rotation2d.fromRotations(0);
  }

  public static class GeneralConstants {}
}
