package team492;

import edu.wpi.first.wpilibj.TimedRobot;
import frclib.FrcCANTalon;
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
    private TrcTalon talon;
    private final double maxVel = 1;
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
        talon = new TrcTalon("talon", 1);
        HalDashboard.putBoolean("useTorque", true);
    }

    @Override
    public void autonomousInit()
    {
        talon.setOdometryEnabled(true);
        talon.enableVelocityMode(maxVel, new TrcPidController.PidCoefficients(0, 0, 0, 1 / maxVel));
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
        talon.set(0.5);
        double time = TrcUtil.getCurrentTime() - startTime;
        double power = talon.getPower();
        double normalizedVel = talon.getVelocity() / maxVel;
        String line = String.format("%.3f,%.2f,%.2f\n", time, power, normalizedVel);
        System.out.print(line);
        if (out != null)
        {
            out.print(line);
        }
    }

    @Override
    public void disabledInit()
    {
        talon.setOdometryEnabled(false);
        talon.disableVelocityMode();
        talon.set(0.0);
        if (out != null)
        {
            out.close();
        }
    }

    @Override
    public void disabledPeriodic()
    {
        talon.set(0.0);
    }

    private class TrcTalon extends TrcMotor
    {
        public FrcCANTalon talon;

        public TrcTalon(String instanceName, int id)
        {
            super(instanceName);
            this.talon = new FrcCANTalon(instanceName, id);
        }

        @Override
        protected double transformTorqueToMotorPower(double percentTorque)
        {
            if (useTorque)
            {
                return super.transformTorqueToMotorPower(percentTorque);
            }
            return percentTorque;
        }

        @Override
        public double getMotorPosition()
        {
            return talon.getMotorPosition();
        }

        @Override
        public double getPosition()
        {
            return talon.getPosition();
        }

        @Override
        public double getVelocity()
        {
            return talon.getVelocity();
        }

        @Override
        public void setMotorPower(double power)
        {
            talon.setMotorPower(power);
        }

        @Override
        public boolean getInverted()
        {
            return talon.getInverted();
        }

        @Override
        public double getPower()
        {
            return talon.getPower();
        }

        @Override
        public boolean isLowerLimitSwitchActive()
        {
            return talon.isLowerLimitSwitchActive();
        }

        @Override
        public boolean isUpperLimitSwitchActive()
        {
            return talon.isUpperLimitSwitchActive();
        }

        @Override
        public void resetPosition(boolean hardware)
        {
            talon.resetPosition(hardware);
        }

        @Override
        public void setBrakeModeEnabled(boolean enabled)
        {
            talon.setBrakeModeEnabled(enabled);
        }

        @Override
        public void setInverted(boolean inverted)
        {
            talon.setInverted(inverted);
        }

        @Override
        public void setPositionSensorInverted(boolean inverted)
        {
            talon.setPositionSensorInverted(inverted);
        }

        @Override
        public void setSoftLimitEnabled(boolean lowerLimitEnabled, boolean upperLimitEnabled)
        {
            talon.setSoftLimitEnabled(lowerLimitEnabled, upperLimitEnabled);
        }

        @Override
        public void setSoftLowerLimit(double position)
        {
            talon.setSoftLowerLimit(position);
        }

        @Override
        public void setSoftUpperLimit(double position)
        {
            talon.setSoftUpperLimit(position);
        }
    }
}
