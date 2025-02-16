package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Manipulator.ManipulatorStates;

public class PassiveIntake extends Command {
    public PassiveIntake() {}

    public void execute() {
        Robot.getManipulator().setManipState(ManipulatorStates.OUT);
    }
    
    public boolean isFinished() {
        return Robot.getManipulator().isCoralReady();
    }

    public void end(boolean interuppted) {
        Robot.getManipulator().setManipState(ManipulatorStates.OFF);
    }
}
