package frc.robot.commands.StateCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.constants.RobotConstants;
import frc.lib.enums.robotStates;
import frc.robot.Robot;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.virtualsubsystems.statehandler.StateHandler;


public class ElevatorToChosenHeight extends Command {
    private final Elevator elevator;
    private final StateHandler stateHandler;

    public ElevatorToChosenHeight(Elevator elevator, StateHandler handler) {
        this.elevator = elevator;
        this.stateHandler = handler;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.elevator);
    }

    @Override
    public void initialize() {
        switch (this.stateHandler.getChosenlevel()) {
            case L1 -> this.stateHandler.setElevatorHeight(RobotConstants.ElevatorConstants.L1height);
            case L2 -> this.stateHandler.setElevatorHeight(RobotConstants.ElevatorConstants.L2height);
            case L3 -> this.stateHandler.setElevatorHeight(RobotConstants.ElevatorConstants.L3height);
            case L4 -> this.stateHandler.setElevatorHeight(RobotConstants.ElevatorConstants.L4height);
        }
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        this.stateHandler.setState(robotStates.RESTING);
    }
}
