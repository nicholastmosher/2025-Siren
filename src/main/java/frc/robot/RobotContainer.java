// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static frc.lib.constants.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.constants.SwerveConstants;
import frc.lib.enums.LevelEnum;
import frc.robot.commands.CommandGroups.IntakeCommandGroup;
import frc.robot.commands.CommandGroups.ScoreCommandGroup;
import frc.robot.commands.Drive.DriveCommands;
import frc.robot.commands.Drive.DriveToPose;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIONeo;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.endeffector.ClawIOVortex;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.endeffector.WristIONeo;
import frc.robot.subsystems.groundintake.GroundIntake;
import frc.robot.subsystems.groundintake.GroundIntakeIOFalconVortex;
import frc.robot.subsystems.groundintake.GroundIntakeIOSim;
import frc.robot.subsystems.virtualsubsystems.statehandler.StateHandler;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final StateHandler stateHandler = new StateHandler();
  private final Drive drive;
  private final Vision vision;
  private final Elevator elevator;
  private final EndEffector endEffector;
  private final GroundIntake groundIntake;

  // Controller
  private final CommandXboxController pilot = new CommandXboxController(0);
  private final CommandXboxController copilot = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  private final IntakeCommandGroup intake;
  private final ScoreCommandGroup score;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(SwerveConstants.FrontLeft),
                new ModuleIOTalonFX(SwerveConstants.FrontRight),
                new ModuleIOTalonFX(SwerveConstants.BackLeft),
                new ModuleIOTalonFX(SwerveConstants.BackRight));

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(camera1Name, robotToCamera1),
                new VisionIOPhotonVision(camera2Name, robotToCamera2),
                new VisionIOPhotonVision(camera3Name, robotToCamera3),
                new VisionIOPhotonVision(camera4Name, robotToCamera4),
                new VisionIOLimelight(limelightName, drive::getRotation));

        elevator = new Elevator(new ElevatorIONeo(), stateHandler);

        groundIntake = new GroundIntake(new GroundIntakeIOFalconVortex(), stateHandler);

        endEffector = new EndEffector(new ClawIOVortex(), new WristIONeo(), stateHandler);

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(SwerveConstants.FrontLeft),
                new ModuleIOSim(SwerveConstants.FrontRight),
                new ModuleIOSim(SwerveConstants.BackLeft),
                new ModuleIOSim(SwerveConstants.BackRight));

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose),
                new VisionIOPhotonVisionSim(camera2Name, robotToCamera2, drive::getPose),
                new VisionIOPhotonVisionSim(camera3Name, robotToCamera3, drive::getPose),
                new VisionIOPhotonVisionSim(camera4Name, robotToCamera4, drive::getPose));

        elevator = new Elevator(new ElevatorIOSim(), stateHandler);

        groundIntake = new GroundIntake(new GroundIntakeIOSim(), stateHandler);

        endEffector = new EndEffector(new ClawIOVortex(), new WristIONeo(), stateHandler);

        break;

      default:
        // Replayed robot, disable IO implementations

        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIO() {},
                new VisionIO() {},
                new VisionIO() {},
                new VisionIO() {},
                new VisionIO() {});

        elevator = new Elevator(new ElevatorIOSim(), stateHandler);

        groundIntake = new GroundIntake(new GroundIntakeIOSim(), stateHandler);

        endEffector = new EndEffector(new ClawIOVortex(), new WristIONeo(), stateHandler);

        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());


    intake = new IntakeCommandGroup(drive, elevator, endEffector, groundIntake, stateHandler);
    score = new ScoreCommandGroup(drive, elevator, endEffector, groundIntake, stateHandler);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive

    double reduction = Math.pow(elevator.getPercentRaised(), 2);
    double cappedreduction = Math.max(reduction, 0.70);

    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive, () -> (pilot.getLeftY()*cappedreduction), () -> (pilot.getLeftX()*cappedreduction), () -> (pilot.getRightX()*cappedreduction)));

    // Reset gyro to 0° when B button is pressed
    pilot
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    pilot.leftTrigger().toggleOnTrue(intake);

    copilot.a().onTrue(new InstantCommand(() -> stateHandler.setLevelEnum(LevelEnum.L1)));
    copilot.b().onTrue(new InstantCommand(() -> stateHandler.setLevelEnum(LevelEnum.L2)));
    copilot.y().onTrue(new InstantCommand(() -> stateHandler.setLevelEnum(LevelEnum.L3)));
    copilot.x().onTrue(new InstantCommand(() -> stateHandler.setLevelEnum(LevelEnum.L4)));

    pilot.rightTrigger().whileTrue(score);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to ru
   *     <p>n in autonomous
   */
  public Command getAutonomousCommand() {
    return new InstantCommand();
  }
}
