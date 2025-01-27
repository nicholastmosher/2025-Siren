package frc.robot.subsystems.claw;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface WristIO {
    @AutoLog
    public static class WristIOInputs {
        
        
    }

    public void setAngle(Rotation2d angle);

    // Updates any periodic logging or state
    public default void updateInputs() {};
}
