package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class PIDY extends Command{
    private SwerveSubsystem swerveSubsystem;
    private double meters;
    
    public PIDY(SwerveSubsystem swerveSubsystem, double meters){
        addRequirements(this.swerveSubsystem);
        this.swerveSubsystem = swerveSubsystem;
        this.meters = meters;
        swerveSubsystem.pidY.setTolerance(0.05);
    }

    @Override
    public void execute(){
        double velocity = swerveSubsystem.pidY.calculate(swerveSubsystem.getFrontMeterValue(), meters);
        swerveSubsystem.drive(0, velocity, 0, false, true);
    }

    @Override
    public void end(boolean interrupted){
        swerveSubsystem.drive(0, 0, 0, false, true);
    }

    @Override
    public boolean isFinished(){
        return swerveSubsystem.pidY.atSetpoint();
    }
}
