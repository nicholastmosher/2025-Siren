package frc.robot.subsystems.groundintake;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  public static class IntakeIOInputs {
    public boolean tiltMotorConnected = false;
    public boolean spinMotorConnected = false;

    public Rotation2d armAngle = new Rotation2d();
  }

  public default void updateInputs(IntakeIOInputs inputs) {}
}
