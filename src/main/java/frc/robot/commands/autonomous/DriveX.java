package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveX extends Command {

    private SwerveSubsystem swerveSubsystem;
    private double speed;

    public DriveX(double speed, SwerveSubsystem swerveSubsystem){
        addRequirements(swerveSubsystem);
        this.swerveSubsystem = swerveSubsystem;
        this.speed = speed;
    }

    @Override
    public void execute(){
        swerveSubsystem.driveX(speed);
    }

    @Override
    public void end(boolean interrupted){
        swerveSubsystem.driveX(0);
    }
}
