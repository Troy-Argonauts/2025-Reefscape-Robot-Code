package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;


import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Handles the manipulator, including motors, sensors, and states.
 * 
 * @author VedNakum, firearcher2012, ASH-will-WIN, Evan13019, shaquilleinoatmeal.
 */
public class Manipulator extends SubsystemBase {
    private DoubleLogEntry topMotorSupplyCurrentLog;
    private DoubleLogEntry bottomMotorSupplyCurrentLog;
    private DoubleLogEntry lateratorMotorSupplyCurrentLog;


    private double lateratorCurrentPosition; 
    private double lateratorTarget;

    private TalonFX topMotor;
    private TalonFX bottomMotor;
    private TalonFX lateratorMotor;
    
    private DigitalInput lateratorLimit;

    private DigitalInput funnelBeamBreak;
    private DigitalInput manipulatorBeamBreak;

    private PositionVoltage positionVoltage = new PositionVoltage(0).withSlot(0);
    
    private TalonFXConfiguration config = new TalonFXConfiguration();

    private Slot0Configs slot0Config = config.Slot0;

    /**
     * Constructor for the Manipulator class. Initializes motors, sensors, and configurations.
     */
    public Manipulator() {
        TalonFXConfiguration topConfiguration = new TalonFXConfiguration();
        TalonFXConfiguration bottomConfiguration = new TalonFXConfiguration();
        TalonFXConfiguration lateratorConfiguration = new TalonFXConfiguration();

        topMotor = new TalonFX(Constants.Manipulator.TOP_MOTOR_CAN_ID);
        bottomMotor = new TalonFX(Constants.Manipulator.BOTTOM_MOTOR_CAN_ID);
        lateratorMotor = new TalonFX(Constants.Manipulator.LATERATOR_MOTOR_CAN_ID);

        lateratorLimit = new DigitalInput(Constants.Manipulator.LATERATOR_LIMIT_SWITCH);

        funnelBeamBreak = new DigitalInput(Constants.Manipulator.FUNNEL_BEAM_BREAK);
        manipulatorBeamBreak = new DigitalInput(Constants.Manipulator.MANIPULATOR_BEAM_BREAK);

        topConfiguration.CurrentLimits.SupplyCurrentLimit = 30;
        bottomConfiguration.CurrentLimits.SupplyCurrentLimit = 30;
        lateratorConfiguration.CurrentLimits.SupplyCurrentLimit = 20;

        topConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        bottomConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        lateratorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        topMotor.getConfigurator().apply(topConfiguration);
        bottomMotor.getConfigurator().apply(bottomConfiguration);
        lateratorMotor.getConfigurator().apply(lateratorConfiguration);

        bottomMotor.setControl(new Follower(Constants.Manipulator.TOP_MOTOR_CAN_ID, false));
        lateratorMotor.getConfigurator().apply(slot0Config);

        DataLog log = DataLogManager.getLog();

        topMotorSupplyCurrentLog = new DoubleLogEntry((log), "Top Manipulator Motor Supply Current");
        bottomMotorSupplyCurrentLog = new DoubleLogEntry(log, "Bottom Manipulator Motor Supply Current");
        lateratorMotorSupplyCurrentLog = new DoubleLogEntry(log, "Laterator Motor Supply Current");

    }

     /**
     * This method is called periodically by the scheduler. Logs current values and updates 
     * SmartDashboard with current and target RPMs.
     */
    @Override
    public void periodic() {

        topMotorSupplyCurrentLog.append(topMotor.getSupplyCurrent().getValueAsDouble());
        bottomMotorSupplyCurrentLog.append(bottomMotor.getSupplyCurrent().getValueAsDouble());   
        lateratorMotorSupplyCurrentLog.append(lateratorMotor.getSupplyCurrent().getValueAsDouble()); 

        SmartDashboard.putBoolean("Coral Ready", isCoralReady());
        SmartDashboard.putNumber("Lat Supply Current", lateratorMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putBoolean("Lat Limit Switch", getLateratorLimit());

        // lateratorMotor.setControl(positionVoletage.withPosition(lateratorTarget));

        // if (getLateratorLimit() == true) {
        //     resetLateratorEncoder();
        // }

        
    }

    /**
     * States of the Manipulator (IN, OUT, OFF)
     */
    public enum ManipulatorStates {
        FORWARD,

        REVERSE,

        OFF;
    }

    /**
     * States of the Laterator (IN, OUT, OFF)
     */
    public enum LateratorStates {
        OFF,

        HOLD,

        OUT;
    }

    /**
     * Sets the laterator's state until it reaches the max laterator position if out or until it reaches 0 if in
     * @param state The desired state of the laterator
     */
    public void setLateratorState(LateratorStates state) {
        switch (state) {
            case OUT: 
                setLateratorRawPower(0.1);
                break;
            case OFF: 
                setLateratorRawPower(0);
                break;
            case HOLD:
                setLateratorRawPower(0.05);
                break;
            
        }
    }

    /**
     * Checks if the limit switch is active.
     * @return True if the limit switch is active, false otherwise.
     */
    public boolean getLateratorLimit(){
        return !lateratorLimit.get();
    }

    /**
     * Returns true if manipulator beam break is being triggered, this will happen when a coral is ready to score in the manipulator.
     * @return true if manipualtor beam break is true, false if manipulator beam break are false
     */
    public boolean isCoralReady() {
        return !manipulatorBeamBreak.get();
    }

    /**
     * Sets the raw power for the laterator motor.
     * @param power The power to set for the laterator motor.
     */
    public void setLateratorRawPower(double power) {
        lateratorMotor.set(power);
    }
 
    /**
     * Sets the manipulator motors state (Expecting manipulater state)
     * @param state the state to set for the manipulator motor
     */
    public void setManipState(ManipulatorStates state) {
        switch (state) {
            case REVERSE: 
                setManipulatorRawPower(-0.30); 
                break;
            case FORWARD: 
                setManipulatorRawPower(0.15);
                break;
            case OFF:
                setManipulatorRawPower(0);
                break;
            
        }
    }

    /**
     * Sets manipulator motors to provided raw power
     * @param speed the raw power to set the motor to
     */
    public void setManipulatorRawPower(double speed){
        topMotor.set(speed);
    }

//     /**
//      * Returns whether a coral has entered the funnel
//      * @return whether coral has entered funnel
//      */
//     public boolean hasCoralEntered() {
//         return funnelBeamBreak.get();
//     }
}