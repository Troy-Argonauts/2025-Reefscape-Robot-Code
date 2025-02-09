import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase{
    private TalonFX armMotor1, armMotor2, alignMotor, tongueMotor;
    private DigitalInput armLimit, tongueLimit, alignLimit;

    public Climber() {}

    @Override
    public void periodic() {
        //needs motors instantiated and methods written
        //ClimberLeftOutputCurrentLog.append(armMotorLeft.getSupplyCurrent().getValueAsDouble());
        //ClimberRightOutputCurrentLog.append(armMotorRight.getSupplyCurrent().getValueAsDouble());
        //ClimberAlignOutputCurrentLog.append(alignMotor.getSupplyCurrent().getValueAsDouble());
        //ClimberToungueOutputCurrentLog.append(tongueMotor.getSupplyCurrent().getValueAsDouble());

        //SmartDashboard.putBoolean("Arm Limit Switch", getArmLimit());
        //SmartDashboard.putBoolean("Tongue Limit Switch", getTongueLimit());
        //SmartDashboard.putBoolean("Alignment Limit Switch", getAlighnmentLimit());
        //SmartDashboard.putNumber("Arm 1 Encoder Position", getArm1EncoderPosition());
        //SmartDashboard.putNumber("Arm 2 Encoder Position", getArm1EncoderPosition());
        //SmartDashboard.putNumber("Alignment Encoder Position", getArm1EncoderPosition());
        //SmartDashboard.putNumber("Tongue Encoder Position", getArm1EncoderPosition());

    }
}

