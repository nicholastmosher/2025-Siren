package frc.robot.subsystems.claw;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Velocity;

public interface ClawIO {
    @AutoLog
    public static class ClawIOInputs {

    }

    public void setSpeed(Velocity speed);

    // Periodic updates for logging and other state handling
    public default void updateInputs() {};
}
