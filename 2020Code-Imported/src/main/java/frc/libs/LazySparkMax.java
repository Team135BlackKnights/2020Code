package frc.libs;

import com.revrobotics.CANSparkMax;

public class LazySparkMax extends CANSparkMax
{
    double upperLimit, lowerLimit;
    IdleMode idleMode;

    public LazySparkMax(int deviceID)
    {   
        super(deviceID, MotorType.kBrushless);
        super.restoreFactoryDefaults();
        idleMode = IdleMode.kBrake;
        super.setIdleMode(idleMode);
        super.setInverted(false);
        upperLimit = 1;
        lowerLimit = 1;
    }

    public LazySparkMax(int deviceID, double upperLimit, double lowerLimit)
    {
        super(deviceID, MotorType.kBrushless);
        super.restoreFactoryDefaults();
        idleMode = IdleMode.kBrake;
        super.setIdleMode(idleMode);
        super.setInverted(false);
        this.upperLimit = upperLimit;
        this.lowerLimit = lowerLimit;
    }

    public LazySparkMax(int deviceID, boolean isBraked, boolean isInverted)
    {
        super(deviceID, MotorType.kBrushless);
        super.restoreFactoryDefaults();
        idleMode = isBraked ? IdleMode.kBrake : IdleMode.kCoast;
        super.setIdleMode(idleMode);
        super.setInverted(isInverted);
        upperLimit = 1;
        lowerLimit = 1;
    }

    public LazySparkMax(int deviceID, boolean isBraked, boolean isInverted, double upperLimit, double lowerLimit)
    {
        super(deviceID, MotorType.kBrushless);
        super.restoreFactoryDefaults();
        idleMode = isBraked ? IdleMode.kBrake : IdleMode.kCoast;
        super.setIdleMode(idleMode);
        super.setInverted(isInverted);
        this.upperLimit = upperLimit;
        this.lowerLimit = lowerLimit;
    }
   
    public void set(double power)
    {
        power = limit(power, upperLimit, lowerLimit);
        super.set(power);
    }

    public double limit(double input, double upperLimit, double lowerLimit)
    {
        if (input >= upperLimit) {
            input = upperLimit;
        } else if (input <= lowerLimit) {
            input = lowerLimit;
        }
        return input;
    }

    public void setSparkLimits(double upperLimit, double lowerLimit)
    {
        this.upperLimit = upperLimit;
        this.lowerLimit = lowerLimit;
    }

    public void setBrakeMode(boolean isBraked)
    {
        idleMode = isBraked ? IdleMode.kBrake : IdleMode.kCoast;
        super.setIdleMode(idleMode);
    }
}