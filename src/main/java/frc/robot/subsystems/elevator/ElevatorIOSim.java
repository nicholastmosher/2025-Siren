package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.util.BasePosition;

public class ElevatorIOSim implements ElevatorIO {

  public ElevatorIOSim() {}

  @Override
  public void move(double input) {}

  @Override
  public void stopElevator() {
    // TODO Auto-generated method stub
  }

  @Override
  public RelativeEncoder getEncoder() {
    // TODO Auto-generated method stub
    return null;
  }

  @Override
  public void resetEncoder() {
    // TODO Auto-generated method stub

  }

  @Override
  public void moveToPoint(Rotation2d targetRot) {
    // TODO Auto-generated method stub

  }

  @Override
  public double getPercentRaised() {
    // TODO Auto-generated method stub
    return 0.0;
  }

  @Override
  public void setTargetPosition(BasePosition position) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setTargetPosition'");
  }

  @Override
  public void periodic() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'periodic'");
  }

  @Override
  public BasePosition getBasePosition() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getBasePosition'");
  }
}
