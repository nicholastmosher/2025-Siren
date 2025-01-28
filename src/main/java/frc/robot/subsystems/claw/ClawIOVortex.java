package frc.robot.subsystems.claw;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.measure.Velocity;

public class ClawIOVortex implements ClawIO {
    
    private final SparkFlex motor;
    private final SparkClosedLoopController m_controller;

    // Current wheel speed (for logging or feedback)
    private double currentSpeed = 0.0;

    // Constructor
    public ClawIOVortex(int motorPort) {
        motor = new SparkFlex(motorPort, MotorType.kBrushless);
        m_controller = motor.getClosedLoopController();

    }

    public void setSpeed(double speed) {
       m_controller.setReference(speed, ControlType.kMAXMotionVelocityControl);
    }

    public double getCurrentSpeed() {
        return motor.getEncoder().getVelocity();
    }

    public void updateInputs(ClawIOInputs inputs) {
        
    }

    @Override
    public void setSpeed(Velocity speed) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setSpeed'");
    }
}
