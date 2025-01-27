package frc.robot.subsystems.claw;

import org.littletonrobotics.junction.Logger;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.measure.Velocity;

public class ClawIOVortex implements ClawIO {
    
    private final SparkFlex motor;

    // Current wheel speed (for logging or feedback)
    private double currentSpeed = 0.0;

    // Constructor
    public ClawIOVortex(int motorPort) {
        motor = new SparkFlex(motorPort, MotorType.kBrushless);
    }

    @Override
    public void setSpeed(Velocity speed) {

    }

    public double getCurrentSpeed() {
        return 0;
    }

    public void updateInputs(ClawIOInputs inputs) {
        
    }
}
