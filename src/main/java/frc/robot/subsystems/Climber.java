package frc.robot.subsystems;

import edu.wpi.first.util.datalog.DoubleLogEntry;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The Climber subsystem controls the motors and sensors associated with the robot's climber mechanism.
 */
public class Climber extends SubsystemBase{
    private TalonFX armMotorLeft, armMotorRight, alignMotor, tongueMotor;
    private DigitalInput armLimit, tongueLimit, alignLimit;

    /**
     * Initializes the Climber subsystem with the motors and sensors.
     */
    public Climber() {
        armMotorLeft = new TalonFX(Constants.Climber.LEFT_MOTOR_ID)
        armMotorRight = new TalonFX(Constants.Climber.RIGHT_MOTOR_ID);
        alignMotor = new TalonFX(Constants.Climber.ALIGN_MOTOR_ID);
        tongueMotor = new TalonFX(Constants.Climber.TONGUE_MOTOR_ID);

        armLimit = new DigitalInput(0);
        tongueLimit = new DigitalInput(1);
        alignLimit = new DigitalInput(2);
        
        armMotorLeft.setNeutralMode(NeutralModeValue.Brake);
        armMotorRight.setNeutralMode(NeutralModeValue.Brake);
        alignMotor.setNeutralMode(NeutralModeValue.Brake);
        tongueMotor.setNeutralMode(NeutralModeValue.Brake);

        TalonFXConfiguration leftMotorConfig = new TalonFXConfiguration();
        TalonFXConfiguration rightMotorConfig = new TalonFXConfiguration();
        TalonFXConfiguration alignMotorConfig = new TalonFXConfiguration();
        TalonFXConfiguration tongueMotorConfig = new TalonFXConfiguration();

        leftMotorConfig.CurrentLimits.SupplyCurrentLimit = 40;
        rightMotorConfig.CurrentLimits.SupplyCurrentLimit = 40;
        alignMotorConfig.CurrentLimits.SupplyCurrentLimit = 30;
        tongueMotorConfig.CurrentLimits.SupplyCurrentLimit = 40;

        armMotorLeft.getConfigurator().apply(leftMotorConfig);
        armMotorRight.getConfigurator().apply(rightMotorConfig);
        alignMotor.getConfigurator().apply(alignMotorConfig);
        tongueMotor.getConfigurator().apply(tongueMotorConfig);
        

        DataLog = DataLogManager.getLog();

        ClimberLeftOutputCurrentLog = new DoubleLogEntry(log, "ClimberLeftOutputCurrent");
        ClimberRightOutputCurrentLog = new DoubleLogEntry(log, "ClimberRightOutputCurrent");
        ClimberAlignOutputCurrentLog = new DoubleLogEntry(log, "ClimberAlignOutputCurrent");
        ClimberTongueOutputCurrentLog = new DoubleLogEntry(log, "ClimberToungueOutputCurrent");

        armMotorLeft.setControl(new Follower(Constants.Climber.LEFT_MOTOR_ID, true));
    }

    /**
     * This method is called periodically and logs motor current values to the dashboard.
     */
    @Override
    public void periodic() {
        //needs motors instantiated and methods written
        ClimberLeftOutputCurrentLog.append(armMotorLeft.getSupplyCurrent().getValueAsDouble());
        ClimberRightOutputCurrentLog.append(armMotorRight.getSupplyCurrent().getValueAsDouble());
        ClimberAlignOutputCurrentLog.append(alignMotor.getSupplyCurrent().getValueAsDouble());
        ClimberToungueOutputCurrentLog.append(tongueMotor.getSupplyCurrent().getValueAsDouble());

        SmartDashboard.putBoolean("Arm Limit Switch", getArmLimit());
        SmartDashboard.putBoolean("Tongue Limit Switch", getTongueLimit());
        SmartDashboard.putBoolean("Alignment Limit Switch", getAlighnmentLimit());
        SmartDashboard.putNumber("Arm 1 Encoder Position", getArm1EncoderPosition());
        SmartDashboard.putNumber("Arm 2 Encoder Position", getArm1EncoderPosition());
        SmartDashboard.putNumber("Alignment Encoder Position", getArm1EncoderPosition());
        SmartDashboard.putNumber("Tongue Encoder Position", getArm1EncoderPosition());
    }

    /**
     * @return the state of the arm limit switch.
     */
    public double getArmLimit() {
        return ArmLimit.get();
    }

    /**
     * @return the state of the tongue limit switch.
     */
    public double getTongueLimit() {
        return TongueLimit.get();
    }

    /**
     * @return the state of the alignment limit switch.
     */
    public double getAlignmentLimit() {
        return AlignmentLimit.get();
    }

    /**
     * States for the climber mechanism.
     */
    public enum ClimberStates{
        IN,

        OUT,

        OFF;
    }

    /**
     * Sets the state of the climber mechanism.
     * 
     * @param state The state to set the climber to.
     */
    public void setState(ClimberStates state) {
        switch (state){
            case IN:
                armMotorLeft.set(0);
                armMotorRight.set(0);
                alignMotor.set(0);
                tongueMotor.set(0);
                break;
            case OFF:
                armMotorLeft.set(0);
                armMotorRight.set(0);
                alignMotor.set(0);
                tongueMotor.set(0);
                break;
            case OUT:
                armMotorLeft.set(0);
                armMotorRight.set(0);
                alignMotor.set(0);
                tongueMotor.set(0);
                break;
        }
        //Change values above
    }

    /**
     * Sets the power of the arm motors.
     * 
     * @param power The power to set the arm motors to.
     */
    public void setRawPower(double power){
        armMotorLeft.set(power);
        armMotorRight.set(power);
    }
}
