package frc.robot.subsystems.claw;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.RobotConstants;

public class WristIONeo implements WristIO {
    private final SparkMax motor;
    private final AbsoluteEncoder encoder;


    // Desired angle for the wrist
    private double desiredAngle = 0.0;

    // Constructor
    public WristIONeo() {
        motor = new SparkMax(RobotConstants.EndEffector.wristmotorID, MotorType.kBrushless);
        encoder = motor.getAbsoluteEncoder();
    }

    @Override
    public void setAngle(Rotation2d angle) {
        
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(encoder.getPosition()); // Adjust based on your encoder configuration
    }

    
    public void updateInputs(WristIOInputs inputs) {
        
    }
}
