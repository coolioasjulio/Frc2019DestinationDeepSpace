package drivesim;

import trclib.TrcMotor;
import trclib.TrcSensor;
import trclib.TrcUtil;

public class SimulatedMotorController extends TrcMotor
{
    private final TrcSensor.SensorData<Double> position;
    private volatile double power;
    private Thread monitorThread;
    private double speed;

    /**
     *
     * @param topSpeed Top speed, in degrees per second
     */
    public SimulatedMotorController(final double topSpeed)
    {
        super("SimulatedMotorController");

        position = new TrcSensor.SensorData<>(TrcUtil.getCurrentTime(),0.0);

        monitorThread = new Thread(() -> {
            while(!Thread.interrupted())
            {
                synchronized (position)
                {
                    double currTime = TrcUtil.getCurrentTime();
                    double positionTime = position.timestamp;
                    if(currTime == positionTime) continue;
                    speed = power * topSpeed;
                    double positionChange = speed * (currTime - positionTime);
                    double newPosition = position.value + positionChange;
                    position.timestamp = currTime;
                    position.value = newPosition;
                }
                try
                {
                    Thread.sleep(50);
                }
                catch (InterruptedException e)
                {
                    break;
                }
            }
        });
        monitorThread.setDaemon(true);
        monitorThread.start();
    }

    public void close()
    {
        monitorThread.interrupt();
    }

    @Override
    public boolean getInverted()
    {
        return false;
    }

    @Override
    public double getMotorPosition()
    {
        return getPosition();
    }

    @Override
    public void setMotorPower(double power)
    {
        set(power);
    }

    @Override
    public double getPosition()
    {
        return position.value;
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
        synchronized (position)
        {
            position.timestamp = TrcUtil.getCurrentTime();
            position.value = 0.0;
        }
    }

    @Override
    public void set(double power)
    {
        this.power = power;
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
