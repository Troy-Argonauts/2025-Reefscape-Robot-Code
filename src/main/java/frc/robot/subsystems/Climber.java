package frc.robot.subsystems;

import edu.wpi.first.util.datalog.DoubleLogEntry;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
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
public class Climber extends SubsystemBase {
    private TalonFX leftArmMotor, rightArmMotor;

    private DoubleLogEntry leftArmMotorPosition;
    private DoubleLogEntry rightArmMotorPosition;
    // private DoubleLogEntry tongueMotorPosition;

    private DoubleLogEntry climberLeftOutputCurrentLog;
    private DoubleLogEntry climberRightOutputCurrentLog;
    // private DoubleLogEntry climberTongueOutputCurrentLog;

    public double armCurrentPosition;
    // public double tongueCurrentPosition;

    /**
    * Initializes the Climber subsystem with the motors and sensors.
    * 
    * @author firearcher2012, Evan13019, shaquilleinoatmeal
    */
    public Climber() {

        leftArmMotor = new TalonFX(Constants.Climber.LEFT_MOTOR_ID);
        rightArmMotor = new TalonFX(Constants.Climber.RIGHT_MOTOR_ID);
        // tongueMotor = new TalonFX(Constants.Climber.TONGUE_MOTOR_ID);

        TalonFXConfiguration leftMotorConfig = new TalonFXConfiguration();
        TalonFXConfiguration rightMotorConfig = new TalonFXConfiguration();
        // TalonFXConfiguration tongueMotorConfig = new TalonFXConfiguration();

        leftMotorConfig.CurrentLimits.SupplyCurrentLimit = 60;
        rightMotorConfig.CurrentLimits.SupplyCurrentLimit = 60;
        // tongueMotorConfig.CurrentLimits.SupplyCurrentLimit = 40;

        leftMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rightMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        rightMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        leftArmMotor.getConfigurator().apply(leftMotorConfig);
        rightArmMotor.getConfigurator().apply(rightMotorConfig);
        // tongueMotor.getConfigurator().apply(tongueMotorConfig);

        // tongueMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;


        DataLog log = DataLogManager.getLog();

        leftArmMotorPosition = new DoubleLogEntry(log, "Left ArmMotor Position");
        rightArmMotorPosition = new DoubleLogEntry(log, "Right ArmMotor Position");
        // tongueMotorPosition = new DoubleLogEntry(log, "Tongue Motor Position");

        climberLeftOutputCurrentLog = new DoubleLogEntry(log, "Climber Left Output Current");
        climberRightOutputCurrentLog = new DoubleLogEntry(log, "Climber Right Output Current");
        // climberTongueOutputCurrentLog = new DoubleLogEntry(log, "Climber Toungue Output Current");

        rightArmMotor.setControl(new Follower(Constants.Climber.LEFT_MOTOR_ID, true));

    }

    /**
    * This method is called periodically and logs motor current values to the dashboard.
    */
    @Override
    public void periodic() {
        leftArmMotorPosition.append(leftArmMotor.getPosition().getValueAsDouble());
        rightArmMotorPosition.append(rightArmMotor.getPosition().getValueAsDouble());
        // tongueMotorPosition.append(tongueMotor.getPosition().getValueAsDouble());

        climberLeftOutputCurrentLog.append(leftArmMotor.getSupplyCurrent().getValueAsDouble());
        climberRightOutputCurrentLog.append(rightArmMotor.getSupplyCurrent().getValueAsDouble());
        // climberTongueOutputCurrentLog.append(tongueMotor.getSupplyCurrent().getValueAsDouble());

        SmartDashboard.putNumber("Arm Encoder Position", rightArmMotor.getPosition().getValueAsDouble());
        // SmartDashboard.putNumber("Tongue Encoder Position", tongueMotor.getPosition().getValueAsDouble());

        
        // tongueCurrentPosition = tongueMotor.getPosition().getValueAsDouble();
        armCurrentPosition = rightArmMotor.getPosition().getValueAsDouble();

        // SmartDashboard.putNumber("Tongue Voltage", tongueMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Arm Voltage", rightArmMotor.getMotorVoltage().getValueAsDouble());

    }

    // /**
    // * States for the Tongue motor.
    // */
    // public enum TongueStates{
    //     OUT,

    //     OFF;
    // }

    /**
    * States for the Arm motor.
    */
    public enum ArmStates{
        IN,

        OUT,

        OFF;
    }

    // /**
    // * Sets the state of the Tongue motor.
    // * 
    // * @param state The state to set the tongue motor to.
    // */
    // public void setTongueState(TongueStates state) {
    //     switch (state){
    //         case OFF:
    //             setTongueRawPower(0);
    //             break;
    //         case OUT:
    //             setTongueRawPower(0.4);
    //             break;
    //     } 
    //     //Change values above
    // }

     /**
    * Sets the state of the climber mechanism.
    * 
    * @param state The state to set the climber to.
    */
    public void setArmState(ArmStates state) {
        switch (state){
            case IN:
                setArmRawPower(-0.15);
                break;
            case OFF:
                setArmRawPower(0);
                break;
            case OUT:
                setArmRawPower(-0.2);//-0.3);
                break;
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

        armCurrentPosition = rightArmMotor.getPosition().getValueAsDouble();

        boolean retval = false;
        if (armCurrentPosition <= Constants.Climber.MAX_ARM_POSITION) {
            retval =  true;
        }

        
        // System.out.println("armExtended =>" + retval + "  " + armCurrentPosition);


        return retval;
    }

    public boolean armRetracted() {
        if (armCurrentPosition <= -0.5) {
            return true;
        }
        return false;
    }

    // public boolean tongueExtended() {
    //     if (tongueCurrentPosition >= Constants.Climber.MAX_Tongue_POSITION) {
    //         return true;
    //     }
    //     return false;
    // }


    /**
     * Resets the climber arm encoder position to zero
     * 
     * @author ASH-will-WIN
     */
    public void resetArmEncoders(){
        leftArmMotor.setPosition(0);
        rightArmMotor.setPosition(0);
    }

    // /**
    //  * Resets the tounge encoder position to zero
    //  * 
    //  * @author ASH-will-WIN
    //  */
    // public void resetTongueEncoder(){
    //     tongueMotor.setPosition(0);
    // }

    /**
     * Sets the arm to a specified raw power
     * @param power power to set the arm motor raw power to
     * 
     * @author ASH-will-WIN
     */
    public void setArmRawPower(double power){
        rightArmMotor.set(power);
        leftArmMotor.set(-power);
    }

    public void setArmPower(double power){
        if (power > 0) {
            rightArmMotor.set(power);
            leftArmMotor.set(-power);
        } else if (power < 0 && getCurrentArmPosition() >= Constants.Climber.MAX_ARM_POSITION) {
            rightArmMotor.set(power);
            leftArmMotor.set(-power);
        } else {
            rightArmMotor.set(0);
            leftArmMotor.set(0);
        }
    }

    // /**
    //  * Sets the tounge to a specified raw power
    //  * @param power power to set the tounge motor raw power to
    //  * 
    //  * @author ASH-will-WIN
    //  */
    // public void setTongueRawPower(double power){
    //     tongueMotor.set(power);
    // }

    /**
     * Returns the current arm motor position
     * @return current arm motor position
     * 
     * @author ASH-will-WIN
     */
    public double getCurrentArmPosition() {
        return armCurrentPosition;
    }

    // public double getCurrentTonguePosition() {
    //     return tongueCurrentPosition;
    // }

    /**
     * Sets arm motor power to the negative of power
     * @param power Input of trigger/joystick, controls power of motors
     */
    public void climberRetract(double power) {
        if (armRetracted() == false) {
            setArmRawPower(power);
        } else {
            setArmRawPower(0);
        }
    }

    // public void adjustSetpoint(double joyStickValue) {
    //     double deadbanded = (Math.abs(joyStickValue) > Constants.Controllers.DEADBAND)
    //                         ? joyStickValue
    //                         : 0;
    //     if (getCurrentTonguePosition() <= Constants.Climber.MAX_Tongue_POSITION){
    //         setTongueRawPower(deadbanded);
    //     } else if (getCurrentTonguePosition() >= Constants.Climber.MAX_Tongue_POSITION) {
    //         setTongueRawPower(deadbanded);
    //     } else {
    //         setTongueRawPower(0);
    //     }
}
