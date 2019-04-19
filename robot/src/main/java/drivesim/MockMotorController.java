package drivesim;

import trclib.TrcMotor;

public class MockMotorController extends TrcMotor
{
    private double position = 0;
    private double velocity = 0;
    private double power;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public MockMotorController(String instanceName)
    {
        super(instanceName);
    }

    public void setPosition(double position)
    {
        this.position = position;
    }

    public void setVelocity(double velocity)
    {
        this.velocity = velocity;
    }

    @Override
    public double getVelocity()
    {
        return velocity;
    }

    @Override
    public double getMotorPosition()
    {
        return position;
    }

    @Override
    public void setMotorPower(double power)
    {
        this.power = power;
    }

    @Override
    public boolean getInverted()
    {
        return false;
    }

    @Override
    public double getPower()
    {
        return power;
    }

    @Override
    public boolean isLowerLimitSwitchActive()
    {
        return false;
    }

    @Override
    public boolean isUpperLimitSwitchActive()
    {
        return false;
    }

    @Override
    public void resetPosition(boolean hardware)
    {

    }

    @Override
    public void setBrakeModeEnabled(boolean enabled)
    {

    }

    @Override
    public void setInverted(boolean inverted)
    {

    }

    @Override
    public void setPositionSensorInverted(boolean inverted)
    {

    }

    @Override
    public void setSoftLimitEnabled(boolean lowerLimitEnabled, boolean upperLimitEnabled)
    {

    }

    @Override
    public void setSoftLowerLimit(double position)
    {

    }

    @Override
    public void setSoftUpperLimit(double position)
    {

    }
}
