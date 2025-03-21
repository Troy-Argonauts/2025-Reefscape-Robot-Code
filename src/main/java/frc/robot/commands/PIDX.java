package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class PIDX extends Command{
    private SwerveSubsystem swerveSubsystem;
    private double meters;
    
    public PIDX(SwerveSubsystem swerveSubsystem, double meters){
        addRequirements(this.swerveSubsystem);
        this.swerveSubsystem = swerveSubsystem;
        this.meters = meters;
        swerveSubsystem.pidX.setTolerance(0.05);
    }

    @Override
    public void execute(){
        double velocity = swerveSubsystem.pidX.calculate(swerveSubsystem.getFrontMeterValue(), meters);
        swerveSubsystem.drive(velocity, 0, 0, false, true);
    }

    @Override
    public void end(boolean interrupted){
        swerveSubsystem.drive(0, 0, 0, false, true);
    }

    @Override
    public boolean isFinished(){
        return swerveSubsystem.pidX.atSetpoint();
    }
}
