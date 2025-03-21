package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class PIDROT extends Command{
    private SwerveSubsystem swerveSubsystem;
    private double angle;
    public double lastGyroAngle;
    
    public PIDROT(SwerveSubsystem swerveSubsystem, double angle, double initGyroAngle){
        addRequirements(this.swerveSubsystem);
        this.swerveSubsystem = swerveSubsystem;
        this.angle = angle;
        swerveSubsystem.pidRot.setTolerance(2);
        lastGyroAngle = initGyroAngle;
    }

    @Override
    public void execute(){
        double correction = swerveSubsystem.pidRot.calculate(swerveSubsystem.getHeading(), lastGyroAngle + angle);
        swerveSubsystem.drive(0, 0, correction, false, true);
    }

    @Override
    public void end(boolean interrupted){
        swerveSubsystem.drive(0, 0, 0, false, true);
    }

    @Override
    public boolean isFinished(){
        return swerveSubsystem.pidRot.atSetpoint();
    }
}
