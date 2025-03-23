package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Tongue extends SubsystemBase{
    private TalonFX tongueMotor;

    private DoubleLogEntry climberTongueOutputCurrentLog;

    private DoubleLogEntry tongueMotorPosition;


    public double tongueCurrentPosition;

    public Tongue() {
        tongueMotor = new TalonFX(Constants.Climber.TONGUE_MOTOR_ID);
        TalonFXConfiguration tongueMotorConfig = new TalonFXConfiguration();
        tongueMotorConfig.CurrentLimits.SupplyCurrentLimit = 40;
        tongueMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        tongueMotor.getConfigurator().apply(tongueMotorConfig);
        DataLog log = DataLogManager.getLog();
        tongueMotorPosition = new DoubleLogEntry(log, "Tongue Motor Position");
        climberTongueOutputCurrentLog = new DoubleLogEntry(log, "Climber Toungue Output Current");
    }

    @Override
    public void periodic() {
        tongueMotorPosition.append(tongueMotor.getPosition().getValueAsDouble());

        climberTongueOutputCurrentLog.append(tongueMotor.getSupplyCurrent().getValueAsDouble());

        tongueCurrentPosition = tongueMotor.getPosition().getValueAsDouble();

        SmartDashboard.putNumber("Tongue Encoder Position", tongueCurrentPosition);        
        SmartDashboard.putNumber("Tongue Voltage", tongueMotor.getMotorVoltage().getValueAsDouble());
    }

     /**
    * States for the Tongue motor.
    */
    public enum TongueStates{
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
            case OFF:
                setTongueRawPower(0);
                break;
            case OUT:
                setTongueRawPower(1);
                break;
        } 
        //Change values above
    }

    public boolean tongueExtended() {
        tongueCurrentPosition = tongueMotor.getPosition().getValueAsDouble();

        boolean retval = false;
        if (tongueCurrentPosition >= Constants.Climber.MAX_Tongue_POSITION) {
            retval =  true;
        }

        
        // System.out.println("armExtended =>" + retval + "  " + armCurrentPosition);


        return retval;
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
     * Sets the tounge to a specified raw power
     * @param power power to set the tounge motor raw power to
     * 
     * @author ASH-will-WIN
     */
    public void setTongueRawPower(double power){
        tongueMotor.set(power);
    }

    public double getCurrentTonguePosition() {
        return tongueCurrentPosition;
    }

    // public void tongueControl(double joyStickValue) {
    //     double deadbanded = (Math.abs(joyStickValue) > Constants.Controllers.DEADBAND)
    //                         ? joyStickValue
    //                         : 0;
    //     if (getCurrentTonguePosition() <= 2.5 && deadbanded > 0){
    //         setTongueRawPower(deadbanded);
    //     } else {
    //         setTongueRawPower(0);
    //     }}
}
