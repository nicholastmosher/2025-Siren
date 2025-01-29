package frc.robot.subsystems.claw;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.RobotConstants;

public class WristIONeo implements WristIO {
    private final SparkMax motor;
    private final SparkClosedLoopController pidController;
    private final AbsoluteEncoder encoder;


    // Desired angle for the wrist
    private double desiredAngle = 0.0;

    // Constructor
    public WristIONeo() {
        motor = new SparkMax(RobotConstants.EndEffector.wristmotorID, MotorType.kBrushless);
        pidController = motor.getClosedLoopController();
        encoder = motor.getAbsoluteEncoder();
    }
///NEEDS TO BE CONFIGURED LOOP
    @Override
    public void setAngle(Rotation2d angle) {
        pidController.setReference(angle.getDegrees(), ControlType.kMAXMotionPositionControl);
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(encoder.getPosition()); // Adjust based on your encoder configuration
    }

    
    public void updateInputs(WristIOInputs inputs) {
        
    }
}
