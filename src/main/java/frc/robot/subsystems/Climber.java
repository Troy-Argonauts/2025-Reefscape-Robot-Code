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
/**
 * The Climber subsystem controls the motors and sensors associated with the robot's climber mechanism.
 * 
 * @author firearcher2012, Evan13019, shaquilleinoatmeal
 */
public class Climber extends SubsystemBase{
    private TalonFX armMotorLeft, armMotorRight, alignMotor, tongueMotor;
    private DigitalInput armLimit, tongueLimit, alignLimit;

    private DoubleLogEntry LeftArmMotorPosition;
    private DoubleLogEntry RightArmMotorPosition;
    private DoubleLogEntry AlignmentMotorPosition;
    private DoubleLogEntry TongueMotorPosition;
    private DoubleLogEntry ClimberLeftOutputCurrentLog;
    private DoubleLogEntry ClimberRightOutputCurrentLog;
    private DoubleLogEntry ClimberAlignOutputCurrentLog;
    private DoubleLogEntry ClimberTongueOutputCurrentLog;


    /**
     * Initializes the Climber subsystem with the motors and sensors.
     */
    public Climber() {
        armMotorLeft = new TalonFX(Constants.Climber.LEFT_MOTOR_ID);
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

        DataLog log = DataLogManager.getLog();

        LeftArmMotorPosition = new DoubleLogEntry(log, "LeftArmMotorPosition");
        RightArmMotorPosition = new DoubleLogEntry(log, "RightArmMotorPosition");
        AlignmentMotorPosition = new DoubleLogEntry(log, "AlignmentMotorPosition");
        TongueMotorPosition = new DoubleLogEntry(log, "TongueMotorPosition");

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
        LeftArmMotorPosition.append(armMotorLeft.getPosition().getValueAsDouble());
        RightArmMotorPosition.append(armMotorRight.getPosition().getValueAsDouble());
        AlignmentMotorPosition.append(alignMotor.getPosition().getValueAsDouble());
        TongueMotorPosition.append(tongueMotor.getPosition().getValueAsDouble());

        ClimberLeftOutputCurrentLog.append(armMotorLeft.getSupplyCurrent().getValueAsDouble());
        ClimberRightOutputCurrentLog.append(armMotorRight.getSupplyCurrent().getValueAsDouble());
        ClimberAlignOutputCurrentLog.append(alignMotor.getSupplyCurrent().getValueAsDouble());
        ClimberTongueOutputCurrentLog.append(tongueMotor.getSupplyCurrent().getValueAsDouble());

        SmartDashboard.putBoolean("Arm Limit Switch", getArmLimit());
        SmartDashboard.putBoolean("Tongue Limit Switch", getTongueLimit());
        SmartDashboard.putBoolean("Alignment Limit Switch", getAlignmentLimit());
        SmartDashboard.putNumber("Arm 1 Encoder Position", armMotorLeft.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Arm 2 Encoder Position", armMotorRight.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Alignment Encoder Position", alignMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Tongue Encoder Position", tongueMotor.getPosition().getValueAsDouble());
    }

    /**
     * @return the state of the arm limit switch.
     */
    public boolean getArmLimit() {
        return armLimit.get();
    }

    /**
     * @return the state of the tongue limit switch.
     */
    public boolean getTongueLimit() {
        return tongueLimit.get();
    }

    /**
     * @return the state of the alignment limit switch.
     */
    public boolean getAlignmentLimit() {
        return alignLimit.get();
    }

    /**
     * States for the Tongue motor.
     */
    public enum TongueStates{
        TONGUE_IN,

        TONGUE_OUT,

        TONGUE_OFF;
    }

    /**
     * States for the Alignment motor.
     */
    public enum AlignStates{
        ALIGN_IN,

        ALIGN_OUT,

        ALIGN_OFF;
    }

    /**
     * States for the Arm motor.
     */
    public enum ArmStates{
        ARM_IN,

        ARM_OUT,

        ARM_OFF;
    }

    /**
     * Sets the state of the Tongue motor.
     * 
     * @param state The state to set the tongue motor to.
     */
    public void setTongueState(TongueStates state) {
        switch (state){
            case TONGUE_IN:
                tongueMotor.set(0);
                break;
            case TONGUE_OFF:
                tongueMotor.set(0);
                break;
            case TONGUE_OUT:
                tongueMotor.set(0);
                break;
        }
        //Change values above
    }

     /**
     * Sets the state of the Alignment motor.
     * 
     * @param state The state to set the alignment motor to.
     */
    public void setAlignState(AlignStates state) {
        switch (state){
            case ALIGN_IN:
                alignMotor.set(0);
                break;
            case ALIGN_OFF:
                alignMotor.set(0);
                break;
            case ALIGN_OUT:
                alignMotor.set(0);
                break;
        }
        //Change values above
    }

     /**
     * Sets the state of the Arm motors.
     * 
     * @param state The state to set the arm motors to.
     */
    public void setArmState(ArmStates state) {
        switch (state){
            case ARM_IN:
                armMotorLeft.set(0);
                break;
            case ARM_OFF:
                armMotorLeft.set(0);
                break;
            case ARM_OUT:
                armMotorLeft.set(0);
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
