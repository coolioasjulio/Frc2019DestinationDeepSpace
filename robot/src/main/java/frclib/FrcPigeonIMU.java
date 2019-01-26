package frclib;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.PigeonIMU;
import trclib.TrcDbgTrace;
import trclib.TrcGyro;
import trclib.TrcUtil;

public class FrcPigeonIMU extends TrcGyro
{
    private PigeonIMU pigeon;
    private double[] ypr = new double[3];

    public FrcPigeonIMU(String instanceName, int id)
    {
        super(instanceName, 3, GYRO_HAS_X_AXIS | GYRO_HAS_Y_AXIS | GYRO_HAS_Z_AXIS);
        pigeon = new PigeonIMU(id);
    }

    @Override
    public SensorData<Double> getRawXData(DataType dataType)
    {
        double time = TrcUtil.getCurrentTime();
        switch (dataType)
        {
            case HEADING:
                return new SensorData<>(time, getPitch());

            case ROTATION_RATE:
                double[] rates = new double[3];
                pigeon.getRawGyro(rates);
                return new SensorData<>(time, rates[1]);

            default:
                throw new UnsupportedOperationException("Unsupported data type!");
        }
    }

    @Override
    public SensorData<Double> getRawYData(DataType dataType)
    {
        double time = TrcUtil.getCurrentTime();
        switch (dataType)
        {
            case HEADING:
                return new SensorData<>(time, getRoll());

            case ROTATION_RATE:
                double[] rates = new double[3];
                pigeon.getRawGyro(rates);
                return new SensorData<>(time, rates[2]);

            default:
                throw new UnsupportedOperationException("Unsupported data type!");
        }
    }

    @Override
    public SensorData<Double> getRawZData(DataType dataType)
    {
        double time = TrcUtil.getCurrentTime();
        switch (dataType)
        {
            case HEADING:
                return new SensorData<>(time, getYaw());

            case ROTATION_RATE:
                double[] rates = new double[3];
                pigeon.getRawGyro(rates);
                return new SensorData<>(time, rates[0]);

            default:
                throw new UnsupportedOperationException("Unsupported data type!");
        }
    }

    public ErrorCode getLastError()
    {
        return pigeon.getLastError();
    }

    public int getDeviceID()
    {
        return pigeon.getDeviceID();
    }

    public double getFusedHeading()
    {
        PigeonIMU.FusionStatus status = new PigeonIMU.FusionStatus();
        pigeon.getFusedHeading(status);
        registerError(status.lastError, instanceName + ".getFusedHeading()");
        return status.heading;
    }

    public double getYaw()
    {
        registerError(pigeon.getYawPitchRoll(ypr), instanceName + ".getYaw()");
        return ypr[0];
    }

    public double getPitch()
    {
        registerError(pigeon.getYawPitchRoll(ypr), instanceName + ".getPitch()");
        return ypr[1];
    }

    public double getRoll()
    {
        registerError(pigeon.getYawPitchRoll(ypr), instanceName + ".getRoll()");
        return ypr[2];
    }

    private void registerError(ErrorCode error, String funcName)
    {
        if (error != ErrorCode.OK)
        {
            TrcDbgTrace.getGlobalTracer()
                .traceErr(funcName, "Pigeon IMU with id %d reported error: %s", pigeon.getDeviceID(), error.toString());
        }
    }
}
