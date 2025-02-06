package frc.robot.subsystems.claw;

import edu.wpi.first.units.measure.Velocity;
import org.littletonrobotics.junction.AutoLog;

public interface ClawIO {
  @AutoLog
  public static class ClawIOInputs {}

  public void setSpeed(Velocity speed);

  // Periodic updates for logging and other state handling
  public default void updateInputs() {}
  ;
}
