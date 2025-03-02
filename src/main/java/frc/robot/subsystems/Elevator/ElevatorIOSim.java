package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class ElevatorIOSim implements ElevatorIO {
  private final ElevatorSim elevatorSim;
  private final PIDController pidController;
  private double height = 0.0;
  private double motorRotation = 0.0;
  private boolean leadConnected = true;
  private boolean followerConnected = true;
  private final LoggedMechanism2d mechanism2d;
  private final LoggedMechanismLigament2d stage;
  private final LoggedMechanismLigament2d carriage;

  public ElevatorIOSim() {
    // Define the system with mass, gearing, drum radius, and motor characteristics
    LinearSystem<N2, N1, N2> elevatorSystem =
        LinearSystemId.createElevatorSystem(
            DCMotor.getNEO(2), 5.0, Units.inchesToMeters(2.0), 0.06666666666);

    elevatorSim =
        new ElevatorSim(
            DCMotor.getNEO(2), // Motor type (2 NEO motors)
            15.0, // Gear ratio (1:15)
            5.0, // Carriage mass in kg
            Units.inchesToMeters(2.0), // Drum radius in meters
            0.0, // Min height (fully retracted)
            2.0, // Max height (fully extended)
            true,
            height,
            null);

    pidController = new PIDController(1.0, 0.0, 0.0); // Example PID values

    // Create a Mechanism2d object for visualization
    mechanism2d = new LoggedMechanism2d(3, 3);
    LoggedMechanismRoot2d root = mechanism2d.getRoot("Elevator Base", 1.5, 0.5);
    stage = root.append(new LoggedMechanismLigament2d("Stage", height / 2, 90));
    carriage = stage.append(new LoggedMechanismLigament2d("Carriage", height / 2, 90));

    // Post to SmartDashboard
    SmartDashboard.putData("Elevator Sim", mechanism2d);
  }

  @Override
  public void move(double input) {
    double pidOutput = pidController.calculate(elevatorSim.getPositionMeters(), input);
    elevatorSim.setInputVoltage(pidOutput * 12.0);
  }

  @Override
  public void stopElevator() {
    elevatorSim.setInputVoltage(0);
  }

  @Override
  public RelativeEncoder getEncoder() {
    return null; // Sim does not use a physical encoder
  }

  @Override
  public void resetEncoder() {}

  @Override
  public void moveToPoint(Rotation2d targetRot) {
    pidController.setSetpoint(targetRot.getRadians());
  }

  @Override
  public double getPercentRaised() {
    return height / 2.0; // Assuming max height of 2 meters
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    elevatorSim.update(0.02); // Simulating periodic update at 20ms intervals
    height = elevatorSim.getPositionMeters();
    stage.setLength(height / 2);
    carriage.setLength(height / 2);

    inputs.axleRotation = Rotation2d.fromRadians(height);
    inputs.motorRotation = Rotation2d.fromRadians(motorRotation);
    inputs.height = height;
    inputs.error = pidController.getAccumulatedError();
    inputs.leadisConnected = leadConnected;
    inputs.followerisConnected = followerConnected;

    // Log mechanism 2D visualization
    Logger.recordOutput("Elevator Mechanism", mechanism2d);
  }
}
