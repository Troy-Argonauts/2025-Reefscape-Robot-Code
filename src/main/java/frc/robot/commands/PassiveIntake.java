package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Manipulator.ManipulatorStates;


/**
 * Sets intake mode to passive and sets the manipulator state to OUT.
 */
public class PassiveIntake extends Command {
    public PassiveIntake() {}

    /**
     * Sets manipulator state to out.
     */
    public void execute() {
        Robot.getManipulator().setManipState(ManipulatorStates.OUT);
    }
    
    /**
     * Returns whether the coral is ready.
     */
    public boolean isFinished() {
        return Robot.getManipulator().isCoralReady();
    }

    /**
     * Sets manipulator state to off. 
     */
    public void end(boolean interuppted) {
        Robot.getManipulator().setManipState(ManipulatorStates.OFF);
    }
}
