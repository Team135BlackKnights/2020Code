package frc.libs;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class LazyTalonFX extends WPI_TalonFX
{
    double upperLimit, lowerLimit;
    NeutralMode brakeMode;
    boolean isInverted, enableVoltageCompensation;

    public LazyTalonFX(int deviceID)
    {
        super(deviceID);
        brakeMode = NeutralMode.Brake;
        upperLimit = 1;
        lowerLimit = 1; 
        isInverted = false;
        enableVoltageCompensation = true;


        super.configPeakOutputForward(upperLimit);
        super.configPeakOutputReverse(lowerLimit);

        super.setNeutralMode(brakeMode);
        super.setInverted(isInverted);

        super.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 100);
        super.configVoltageCompSaturation(12.0, 100);
        super.enableVoltageCompensation(enableVoltageCompensation);
        super.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, 100);
        super.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_50Ms, 100);
        super.configVelocityMeasurementWindow(4, 100);
    }

    public LazyTalonFX(int deviceID, boolean isBraked)
    {
        super(deviceID);
        brakeMode = isBraked ? NeutralMode.Brake : NeutralMode.Coast;
        upperLimit = 1;
        lowerLimit = 1; 
        isInverted = false;
        enableVoltageCompensation = true;


        super.configPeakOutputForward(upperLimit);
        super.configPeakOutputReverse(lowerLimit);

        super.setNeutralMode(brakeMode);
        super.setInverted(isInverted);

        super.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 100);
        super.configVoltageCompSaturation(12.0, 100);
        super.enableVoltageCompensation(enableVoltageCompensation);
        super.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, 100);
        super.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_50Ms, 100);
        super.configVelocityMeasurementWindow(4, 100);
    }

    public LazyTalonFX(int deviceID, boolean isBraked, boolean isInverted)
    {
        super(deviceID);
        brakeMode = isBraked ? NeutralMode.Brake : NeutralMode.Coast;
        upperLimit = 1;
        lowerLimit = 1; 
        this.isInverted = isInverted;
        enableVoltageCompensation = true;


        super.configPeakOutputForward(upperLimit);
        super.configPeakOutputReverse(lowerLimit);

        super.setNeutralMode(brakeMode);
        super.setInverted(isInverted);

        super.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 100);
        super.configVoltageCompSaturation(12.0, 100);
        super.enableVoltageCompensation(enableVoltageCompensation);
        super.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, 100);
        super.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_50Ms, 100);
        super.configVelocityMeasurementWindow(4, 100);
  
    }

    public LazyTalonFX(int deviceID, boolean isBraked, boolean isInverted, double upperLimit, double lowerLimit)
    {
        super(deviceID);
        brakeMode = isBraked ? NeutralMode.Brake : NeutralMode.Coast;
        this.upperLimit = upperLimit;
        this.lowerLimit = lowerLimit; 
        this.isInverted = isInverted;
        enableVoltageCompensation = true;


        super.configPeakOutputForward(upperLimit);
        super.configPeakOutputReverse(lowerLimit);

        super.setNeutralMode(brakeMode);
        super.setInverted(isInverted);

        super.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 100);
        super.configVoltageCompSaturation(12.0, 100);
        super.enableVoltageCompensation(enableVoltageCompensation);
        super.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, 100);
        super.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_50Ms, 100);
        super.configVelocityMeasurementWindow(4, 100);
  
    }



    public void setLimits(double upperLimit, double lowerLimit)
    {
        this.upperLimit = upperLimit;
        this.lowerLimit = lowerLimit;
    }

    public void setBrakeMode(boolean isBraked)
    {
        brakeMode = isBraked ? NeutralMode.Brake : NeutralMode.Coast;
        super.setNeutralMode(brakeMode);
    }
    
}