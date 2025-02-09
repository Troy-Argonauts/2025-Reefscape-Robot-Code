import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase{
    private TalonFX armMotor1, armMotor2, alignMotor, tongueMotor;
    private DigitalInput armLimit, tongueLimit, alignLimit;

    private DoubleLogEntry armMotor1SupplyCurrentLog;
    private DoubleLogEntry armMotor2SupplyCurrentLog;
    private DoubleLogEntry alignMotorSupplyCurrentLog;
    private DoubleLogEntry tongueMotorSupplyCurrentLog;

    public Climber() {}

    @Override
    public void periodic() {
        //needs motors instantiated and methods written
        armMotor1SupplyCurrentLog.append(armMotor1.getSupplyCurrent().getValueAsDouble());
        armMotor2SupplyCurrentLog.append(armMotor2.getSupplyCurrent().getValueAsDouble());
        alignMotorSupplyCurrentLog.append(alignMotor.getSupplyCurrent().getValueAsDouble());
        tongueMotorSupplyCurrentLog.append(tongueMotor.getSupplyCurrent().getValueAsDouble());

        SmartDashboard.putBoolean("Arm Limit Switch", getArmLimit());
        SmartDashboard.putBoolean("Tongue Limit Switch", getTongueLimit());
        SmartDashboard.putBoolean("Alignment Limit Switch", getAlighnmentLimit());
        SmartDashboard.putNumber("Arm 1 Encoder Position", getArm1EncoderPosition());
        SmartDashboard.putNumber("Arm 2 Encoder Position", getArm1EncoderPosition());
        SmartDashboard.putNumber("Alignment Encoder Position", getArm1EncoderPosition());
        SmartDashboard.putNumber("Tongue Encoder Position", getArm1EncoderPosition());
    }
}

