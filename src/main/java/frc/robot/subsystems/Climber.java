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

    private DoubleLogEntry leftArmMotorPosition;
    private DoubleLogEntry rightArmMotorPosition;
    private DoubleLogEntry alignmentMotorPosition;
    private DoubleLogEntry tongueMotorPosition;

    private DoubleLogEntry climberLeftOutputCurrentLog;
    private DoubleLogEntry climberRightOutputCurrentLog;
    private DoubleLogEntry climberAlignOutputCurrentLog;
    private DoubleLogEntry climberTongueOutputCurrentLog;

    public double armCurrentPosition;
    public double tongueCurrentPosition;
    public double alignCurrentPosition;


    /**
    * Initializes the Climber subsystem with the motors and sensors.
    * 
    * @author firearcher2012, Evan13019, shaquilleinoatmeal
    */
    public Climber() {
        armMotorLeft = new TalonFX(Constants.Climber.LEFT_MOTOR_ID);
        armMotorRight = new TalonFX(Constants.Climber.RIGHT_MOTOR_ID);
        alignMotor = new TalonFX(Constants.Climber.ALIGN_MOTOR_ID);
        tongueMotor = new TalonFX(Constants.Climber.TONGUE_MOTOR_ID);

        armLimit = new DigitalInput(Constants.Climber.ARM_LIMIT_SWITCH);
        tongueLimit = new DigitalInput(Constants.Climber.TONGUE_LIMIT_SWITCH);
        alignLimit = new DigitalInput(Constants.Climber.ALIGN_LIMIT_SWITCH);
        
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

        leftArmMotorPosition = new DoubleLogEntry(log, "Left ArmMotor Position");
        rightArmMotorPosition = new DoubleLogEntry(log, "Right ArmMotor Position");
        alignmentMotorPosition = new DoubleLogEntry(log, "Alignment Motor Position");
        tongueMotorPosition = new DoubleLogEntry(log, "Tongue Motor Position");

        climberLeftOutputCurrentLog = new DoubleLogEntry(log, "Climber Left Output Current");
        climberRightOutputCurrentLog = new DoubleLogEntry(log, "Climber Right Output Current");
        climberAlignOutputCurrentLog = new DoubleLogEntry(log, "Climber Align Output Current");
        climberTongueOutputCurrentLog = new DoubleLogEntry(log, "Climber Toungue Output Current");

        armMotorRight.setControl(new Follower(Constants.Climber.LEFT_MOTOR_ID, true));
    }

    /**
    * This method is called periodically and logs motor current values to the dashboard.
    */
    @Override
    public void periodic() {
        leftArmMotorPosition.append(armMotorLeft.getPosition().getValueAsDouble());
        rightArmMotorPosition.append(armMotorRight.getPosition().getValueAsDouble());
        alignmentMotorPosition.append(alignMotor.getPosition().getValueAsDouble());
        tongueMotorPosition.append(tongueMotor.getPosition().getValueAsDouble());

        climberLeftOutputCurrentLog.append(armMotorLeft.getSupplyCurrent().getValueAsDouble());
        climberRightOutputCurrentLog.append(armMotorRight.getSupplyCurrent().getValueAsDouble());
        climberAlignOutputCurrentLog.append(alignMotor.getSupplyCurrent().getValueAsDouble());
        climberTongueOutputCurrentLog.append(tongueMotor.getSupplyCurrent().getValueAsDouble());

        SmartDashboard.putBoolean("Arm Limit Switch", getArmLimit());
        SmartDashboard.putBoolean("Tongue Limit Switch", getTongueLimit());
        SmartDashboard.putBoolean("Alignment Limit Switch", getAlignmentLimit());
        SmartDashboard.putNumber("Arm 1 Encoder Position", armMotorLeft.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Arm 2 Encoder Position", armMotorRight.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Alignment Encoder Position", alignMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Tongue Encoder Position", tongueMotor.getPosition().getValueAsDouble());

        run(null);

        if (getArmLimit() == true) {
            resetArmEncoder();
        }

        if (getAlignmentLimit() == true) {
            resetAlignEncoder();
        }

        if (getTongueLimit() == true) {
            resetTongueEncoder();
        }
        
    }

    /**
    * This method retrieves the current state of the arm limit switch.
    *
    * @return a boolean value representing the state of the arm limit switch;
    *         true if the limit switch is activated, false otherwise.
    */
    public boolean getArmLimit() {
        return armLimit.get();
    }

    /**
    * This method retrieves the current state of the tongue limit switch.
    *
    * @return a boolean value representing the state of the tongue limit switch;
    *         true if the limit switch is activated, false otherwise.
    */
    public boolean getTongueLimit() {
        return tongueLimit.get();
    }

    /**
    * This method retrieves the current state of the alignment limit switch.
    *
    * @return a boolean value representing the state of the alignment limit switch; 
    *         true if the limit switch is activated, false otherwise.
    */
    public boolean getAlignmentLimit() {
        return alignLimit.get();
    }

    /**
    * States for the Tongue motor.
    */
    public enum TongueStates{
        IN,

        OUT,

        OFF;
    }

    /**
    * States for the Alignment motor.
    */
    public enum AlignStates{
        IN,

        OUT,

        OFF;
    }

    /**
    * States for the Arm motor.
    */
    public enum ArmStates{
        IN,

        OUT,

        OFF;
    }

    /**
    * Sets the state of the Tongue motor.
    * 
    * @param state The state to set the tongue motor to.
    */
    public void setTongueState(TongueStates state) {
        switch (state){
            case IN:
                setTongueRawPower(-0.3);
            case OFF:
                setTongueRawPower(0.3);
            case OUT:
                setTongueRawPower(0);
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
            case IN:
                setAlignRawPower(-0.3);
            case OFF:
                setAlignRawPower(0.3);
            case OUT:
                setAlignRawPower(0);
        }
        //Change values above
    }

     /**
    * Sets the state of the climber mechanism.
    * 
    * @param state The state to set the climber to.
    */
    public void setArmState(ArmStates state) {
        switch (state){
            case IN:
                setArmRawPower(-0.3);
            case OFF:
                setArmRawPower(0.3);
            case OUT:
                setArmRawPower(0);
        }
        //Change values above
    }

    public boolean ArmExtended() {
        if (armMotorLeft.getPosition().getValueAsDouble() >= Constants.Climber.MAX_ARM_POSITION) {
            return true;
        }
        return false;
    }

    public boolean TongueExtended() {
        if (tongueMotor.getPosition().getValueAsDouble() >= Constants.Climber.MAX_Tongue_POSITION) {
            return true;
        }
        return false;
    }

    public boolean AlignExtended() {
        if (alignMotor.getPosition().getValueAsDouble() >= Constants.Climber.MAX_Align_POSITION) {
            return true;
        }
        return false;
    }

    public void resetArmEncoder(){
        armMotorLeft.setPosition(0);
        armMotorRight.setPosition(0);
    }

    public void resetAlignEncoder(){
        alignMotor.setPosition(0);
    }

    public void resetTongueEncoder(){
        tongueMotor.setPosition(0);
    }

    
    public void setArmRawPower(double power){
        armMotorRight.set(power);
        armMotorLeft.set(power);
    }

    public void setTongueRawPower(double power){
        tongueMotor.set(power);
    }

    public void setAlignRawPower(double power){
        alignMotor.set(power);
    }

    public double getCurrentTonguePosition() {
        return tongueCurrentPosition;
    }

    public double getCurrentAlignPosition() {
        return alignCurrentPosition;
    }

    public double getCurrentArmPosition() {
        return armCurrentPosition;
    }
}
