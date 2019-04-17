package drivesim;

import trclib.TrcGyro;
import trclib.TrcUtil;

public class MockGyro extends TrcGyro
{
    public MockGyro(String instanceName)
    {
        super(instanceName, 3,
            TrcGyro.GYRO_HAS_X_AXIS | TrcGyro.GYRO_HAS_Y_AXIS | TrcGyro.GYRO_HAS_Z_AXIS);
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
        return new SensorData<>(TrcUtil.getCurrentTime(), 0.0);
    }
}
