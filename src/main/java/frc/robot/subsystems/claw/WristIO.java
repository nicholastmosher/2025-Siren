package frc.robot.subsystems.claw;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
  @AutoLog
  public static class WristIOInputs {}

  public void setAngle(Rotation2d angle);

  // Updates any periodic logging or state
  public default void updateInputs() {}
  ;
}
