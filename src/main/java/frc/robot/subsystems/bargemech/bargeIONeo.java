package frc.robot.subsystems.bargemech;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.constants.RobotConstants;
import org.littletonrobotics.junction.Logger;

public class bargeIONeo implements bargeIO {
  private final SparkMax motor1;
  private final SparkMax motor2;

  // Constructor
  public bargeIONeo() {

    motor1 = new SparkMax(35, MotorType.kBrushless);
    motor2 = new SparkMax(36, MotorType.kBrushless);
  }


  @Override
  public void intake(double speed) {
    motor1.set(speed);
    motor2.set(-speed);
  }

  @Override
  public void outake(double speed) {
    motor1.set(-speed);
    motor2.set(speed);
  }



  @Override
  public void stopMotor() {
    motor1.stopMotor();
    motor2.stopMotor();
  }




}
