package drivesim;

import trclib.TrcGyro;
import trclib.TrcUtil;

public class MockGyro extends TrcGyro
{
    private double heading;
    private double rotationalVelocity;
    private Double lastTime;

    public MockGyro(String instanceName)
    {
        super(instanceName, 1, TrcGyro.GYRO_HAS_Z_AXIS);
    }

    public void setHeading(double heading)
    {
        double currTime = TrcUtil.getCurrentTime();
        if (lastTime != null)
        {
            rotationalVelocity = (heading - this.heading) / (currTime - lastTime);
        }
        lastTime = currTime;
        this.heading = heading;
    }

    @Override
    public SensorData<Double> getRawXData(DataType dataType)
    {
        return new SensorData<>(TrcUtil.getCurrentTime(), 0.0);
    }

    @Override
    public SensorData<Double> getRawYData(DataType dataType)
    {
        return new SensorData<>(TrcUtil.getCurrentTime(), 0.0);
    }

    @Override
    public SensorData<Double> getRawZData(DataType dataType)
    {
        return new SensorData<>(TrcUtil.getCurrentTime(), dataType == DataType.HEADING ? heading : rotationalVelocity);
    }
}
