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
* @author ASH-will-WIN, firearcher2012, Evan13019, shaquilleinoatmeal, sanjayshank
*/
public class Climber extends SubsystemBase{
    private TalonFX leftArmMotor, rightArmMotor, tongueMotor;

    private DigitalInput armLimit, tongueLimit;

    private DoubleLogEntry leftArmMotorPosition;
    private DoubleLogEntry rightArmMotorPosition;
    private DoubleLogEntry tongueMotorPosition;

    private DoubleLogEntry climberLeftOutputCurrentLog;
    private DoubleLogEntry climberRightOutputCurrentLog;
    private DoubleLogEntry climberTongueOutputCurrentLog;

    public double armCurrentPosition;
    public double tongueCurrentPosition;


    /**
    * Initializes the Climber subsystem with the motors and sensors.
    * 
    * @author firearcher2012, Evan13019, shaquilleinoatmeal
    */
    public Climber() {
        leftArmMotor = new TalonFX(Constants.Climber.LEFT_MOTOR_ID);
        rightArmMotor = new TalonFX(Constants.Climber.RIGHT_MOTOR_ID);
        tongueMotor = new TalonFX(Constants.Climber.TONGUE_MOTOR_ID);

        armLimit = new DigitalInput(Constants.Climber.ARM_LIMIT_SWITCH);
        tongueLimit = new DigitalInput(Constants.Climber.TONGUE_LIMIT_SWITCH);
        
        leftArmMotor.setNeutralMode(NeutralModeValue.Brake);
        rightArmMotor.setNeutralMode(NeutralModeValue.Brake);
        tongueMotor.setNeutralMode(NeutralModeValue.Brake);

        TalonFXConfiguration leftMotorConfig = new TalonFXConfiguration();
        TalonFXConfiguration rightMotorConfig = new TalonFXConfiguration();
        TalonFXConfiguration tongueMotorConfig = new TalonFXConfiguration();

        leftMotorConfig.CurrentLimits.SupplyCurrentLimit = 40;
        rightMotorConfig.CurrentLimits.SupplyCurrentLimit = 40;
        tongueMotorConfig.CurrentLimits.SupplyCurrentLimit = 40;

        leftArmMotor.getConfigurator().apply(leftMotorConfig);
        rightArmMotor.getConfigurator().apply(rightMotorConfig);
        tongueMotor.getConfigurator().apply(tongueMotorConfig);

        DataLog log = DataLogManager.getLog();

        leftArmMotorPosition = new DoubleLogEntry(log, "Left ArmMotor Position");
        rightArmMotorPosition = new DoubleLogEntry(log, "Right ArmMotor Position");
        tongueMotorPosition = new DoubleLogEntry(log, "Tongue Motor Position");

        climberLeftOutputCurrentLog = new DoubleLogEntry(log, "Climber Left Output Current");
        climberRightOutputCurrentLog = new DoubleLogEntry(log, "Climber Right Output Current");
        climberTongueOutputCurrentLog = new DoubleLogEntry(log, "Climber Toungue Output Current");

        rightArmMotor.setControl(new Follower(Constants.Climber.LEFT_MOTOR_ID, true));
    }

    /**
    * This method is called periodically and logs motor current values to the dashboard.
    */
    @Override
    public void periodic() {
        leftArmMotorPosition.append(leftArmMotor.getPosition().getValueAsDouble());
        rightArmMotorPosition.append(rightArmMotor.getPosition().getValueAsDouble());
        tongueMotorPosition.append(tongueMotor.getPosition().getValueAsDouble());

        climberLeftOutputCurrentLog.append(leftArmMotor.getSupplyCurrent().getValueAsDouble());
        climberRightOutputCurrentLog.append(rightArmMotor.getSupplyCurrent().getValueAsDouble());
        climberTongueOutputCurrentLog.append(tongueMotor.getSupplyCurrent().getValueAsDouble());

        SmartDashboard.putBoolean("Arm Limit Switch", getArmLimit());
        SmartDashboard.putBoolean("Tongue Limit Switch", getTongueLimit());
        SmartDashboard.putNumber("Arm 1 Encoder Position", leftArmMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Arm 2 Encoder Position", rightArmMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Tongue Encoder Position", tongueMotor.getPosition().getValueAsDouble());

        
        tongueCurrentPosition = tongueMotor.getPosition().getValueAsDouble();
        armCurrentPosition = leftArmMotor.getPosition().getValueAsDouble();
        if (getArmLimit() == true) {
            resetArmEncoders();
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
    * @author ASH-will-WIN, firearcher2012, Evan13019, shaquilleinoatmeal, sanjayshank
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
    * States for the Tongue motor.
    */
    public enum TongueStates{
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

    /**
     * Returns whether the climber arm is extended or not
     * 
     * @return whether the dlimber arm is extended
     * @author ASH-will-WIN
     */
    public boolean armExtended() {
        if (armCurrentPosition >= Constants.Climber.MAX_ARM_POSITION) {
            return true;
        }
        return false;
    }

    /**
     * Returns whether the tounge is extended or not
     * 
     * @return whether the tounge is extended or not
     * 
     */
    public boolean tongueExtended() {
        if (tongueCurrentPosition >= Constants.Climber.MAX_Tongue_POSITION) {
            return true;
        }
        return false;
    }

    /**
     * Resets the climber arm encoder position to zero
     * 
     * @author ASH-will-WIN
     */
    public void resetArmEncoders(){
        leftArmMotor.setPosition(0);
        rightArmMotor.setPosition(0);
    }

    /**
     * Resets the tounge encoder position to zero
     * 
     * @author ASH-will-WIN
     */
    public void resetTongueEncoder(){
        tongueMotor.setPosition(0);
    }

    /**
     * Sets the arm to a specified raw power
     * @param power power to set the arm motor raw power to
     * 
     * @author ASH-will-WIN
     */
    public void setArmRawPower(double power){
        leftArmMotor.set(power);
        rightArmMotor.set(power);
    }

    /**
     * Sets the tounge to a specified raw power
     * @param power power to set the tounge motor raw power to
     * 
     * @author ASH-will-WIN
     */
    public void setTongueRawPower(double power){
        tongueMotor.set(power);
    }

    /**
     * Returns the current tounge motor position
     * @return current tounge motor position
     * 
     * @author ASH-will-WIN
     */
    public double getCurrentTonguePosition() {
        return tongueCurrentPosition;
    }

    /**
     * Returns the current arm motor position
     * @return current arm motor position
     * 
     * @author ASH-will-WIN
     */
    public double getCurrentArmPosition() {
        return armCurrentPosition;
    }

    /**
     * Sets arm motor power to the negative of power
     * @param power Input of trigger/joystick, controls power of motors
     */
    public void climberRetract(double power) {
        if (getArmLimit() == false) {
            setArmRawPower(-Math.abs(power));
        } else {
            setArmRawPower(0);
        }
    }
}
