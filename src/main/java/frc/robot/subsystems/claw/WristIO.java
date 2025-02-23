package frc.robot.subsystems.claw;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
  @AutoLog
  public static class WristIOInputs {
    double angle = 0;
    ;
  }

  public void setAngle(Rotation2d angle);

  public void stopMotor();

  // Updates any periodic logging or state
  public default void updateInputs() {}
  ;
}
