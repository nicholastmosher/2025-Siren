package frc.robot.subsystems.groundintake;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface IntakeIO {

    @AutoLog
    public static class IntakeIOInputs {
        public boolean tiltMotorConnected =false;
        public boolean spinMotorConnected = false;

        public Rotation2d armAngle = new Rotation2d();
        
    }


    public default void updateInputs(IntakeIOInputs inputs) {}
    
}
