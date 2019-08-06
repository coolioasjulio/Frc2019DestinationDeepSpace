package team492;

import edu.wpi.first.wpilibj.TimedRobot;
import frclib.FrcCANSparkMax;
import hallib.HalDashboard;
import trclib.TrcMotor;
import trclib.TrcPidController;
import trclib.TrcUtil;

import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.PrintStream;
import java.text.SimpleDateFormat;
import java.util.Date;

public class PlaygroundRobot extends TimedRobot
{
    private TrcSpark spark;
    private final double maxVel = 5676.0 / 60.0;
    private boolean useTorque = true;
    private double startTime;
    private PrintStream out;

    public PlaygroundRobot()
    {
        super(0.01);
    }

    @Override
    public void robotInit()
    {
        spark = new TrcSpark("spark", 4);
        HalDashboard.putBoolean("useTorque", true);
        HalDashboard.putNumber("vel", 0);
    }

    @Override
    public void autonomousInit()
    {
        spark.setOdometryEnabled(true);
        spark.enableVelocityMode(maxVel, new TrcPidController.PidCoefficients(0, 0, 0, 1.0));
        startTime = TrcUtil.getCurrentTime();
        useTorque = HalDashboard.getBoolean("useTorque", true);
        try
        {
            SimpleDateFormat format = new SimpleDateFormat("MMM-dd_HH-mm");
            Date date = new Date(System.currentTimeMillis());
            out = new PrintStream(new FileOutputStream(String.format("home/lvuser/test_%s.csv", format.format(date))));
            out.println("Time,Power,NormalizedVel");
        }
        catch (FileNotFoundException e)
        {
            e.printStackTrace();
        }
    }

    @Override
    public void autonomousPeriodic()
    {
        spark.set(0.5);
        double time = TrcUtil.getCurrentTime() - startTime;
        double power = spark.getPower();
        double normalizedVel = spark.getVelocity() / maxVel;
        String line = String.format("%.3f,%.2f,%.2f\n", time, power, normalizedVel);
        HalDashboard.putNumber("vel", normalizedVel);
        System.out.print(line);
        if (out != null)
        {
            out.print(line);
        }
    }

    @Override
    public void disabledInit()
    {
        spark.setOdometryEnabled(false);
        spark.disableVelocityMode();
        spark.set(0.0);
        if (out != null)
        {
            out.close();
        }
    }

    @Override
    public void disabledPeriodic()
    {
        spark.set(0.0);
    }

    /**
     * This is just a wrapper that's basically a partial implementation of FrcCANSparkMax. This is because
     * we want to use the TrcMotor velocity control code. This was initially made for talons, since the talon
     * implements native velocity control. The spark doesn't, but I didn't want to scrap this work, so whatever.
     */
    private class TrcSpark extends TrcMotor
    {
        public FrcCANSparkMax spark;

        public TrcSpark(String instanceName, int id)
        {
            super(instanceName);
            this.spark = new FrcCANSparkMax(instanceName, id, true);
            this.spark.motor.disableVoltageCompensation();
        }

        @Override
        protected double transformTorqueToMotorPower(double percentTorque)
        {
            System.out.println(percentTorque);
            if (useTorque)
            {
                return super.transformTorqueToMotorPower(percentTorque);
            }
            return percentTorque;
        }

        @Override
        public double getMotorPosition()
        {
            return spark.getMotorPosition();
        }

        @Override
        public double getPosition()
        {
            return spark.getPosition();
        }

        @Override
        public double getVelocity()
        {
            return spark.getVelocity();
        }

        @Override
        public void setMotorPower(double power)
        {
            spark.setMotorPower(power);
        }

        @Override
        public boolean getInverted()
        {
            return spark.getInverted();
        }

        @Override
        public double getPower()
        {
            return spark.getPower();
        }

        @Override
        public boolean isLowerLimitSwitchActive()
        {
            return spark.isLowerLimitSwitchActive();
        }

        @Override
        public boolean isUpperLimitSwitchActive()
        {
            return spark.isUpperLimitSwitchActive();
        }

        @Override
        public void resetPosition(boolean hardware)
        {
            spark.resetPosition(hardware);
        }

        @Override
        public void setBrakeModeEnabled(boolean enabled)
        {
            spark.setBrakeModeEnabled(enabled);
        }

        @Override
        public void setInverted(boolean inverted)
        {
            spark.setInverted(inverted);
        }

        @Override
        public void setPositionSensorInverted(boolean inverted)
        {
            spark.setPositionSensorInverted(inverted);
        }

        @Override
        public void setSoftLimitEnabled(boolean lowerLimitEnabled, boolean upperLimitEnabled)
        {
            spark.setSoftLimitEnabled(lowerLimitEnabled, upperLimitEnabled);
        }

        @Override
        public void setSoftLowerLimit(double position)
        {
            spark.setSoftLowerLimit(position);
        }

        @Override
        public void setSoftUpperLimit(double position)
        {
            spark.setSoftUpperLimit(position);
        }
    }
}
