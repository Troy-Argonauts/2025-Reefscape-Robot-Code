package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ClimberRun extends Command{

    private Climber climber;
    private double positiveSpeed, negativeSpeed;

    public ClimberRun(Climber climber,double positiveSpeed, double negativeSpeed) {
        addRequirements(climber);
        this.climber = climber;
        this.positiveSpeed = positiveSpeed;
        this.negativeSpeed = negativeSpeed;
    }

    @Override
    public void execute() {
        if(positiveSpeed > 0) {
            climber.climberRetract(positiveSpeed);
        }else if(negativeSpeed > 0) {
            climber.climberRetract(negativeSpeed);
        }else {
            climber.climberRetract(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        climber.climberRetract(0);
    }
}
