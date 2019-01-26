package team492;

import frclib.FrcMotionMagicRotationController;
import hallib.HalDashboard;
import trclib.TrcEvent;
import trclib.TrcPidController;
import trclib.TrcRobot;
import trclib.TrcUtil;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.PrintStream;
import java.text.SimpleDateFormat;
import java.util.Date;

public class MotionMagicRotTest implements TrcRobot.RobotCommand
{
    private static final double kP = 0.02;
    private static final double kI = 0.0;
    private static final double kD = 0.00175;
    private static final double kF = 1.131266385; // TODO: Calculate this according to Phoenix docs

    private static final double MAX_SPEED = 372; // deg/sec
    private static final double MAX_ACCEL = 300; // deg/sec^2

    private static final boolean WRITE_CSV = true;

    private FrcMotionMagicRotationController motionMagic;
    private Robot robot;
    private TrcEvent event;
    private double startTime;
    private PrintStream fileOut;
    private double targetAngle;

    public MotionMagicRotTest(Robot robot)
    {
        this.robot = robot;
        TrcPidController.PidCoefficients pidCoefficients = new TrcPidController.PidCoefficients(kP, kI, kD, kF);
        this.motionMagic = new FrcMotionMagicRotationController("MotionMagic", pidCoefficients, robot.pigeon, MAX_SPEED,
            MAX_ACCEL, 2.0);
        motionMagic.setAutoTimeoutEnabled(true, 1.5);
        motionMagic.setLeftMotors(robot.leftFrontWheel, robot.leftRearWheel);
        motionMagic.setRightMotors(robot.rightFrontWheel, robot.rightRearWheel);

        event = new TrcEvent("MotionMagicTest.TrcEvent");

        refreshData("Test/TurnDegrees", 0.0);
    }

    private void refreshData(String name, double defaultValue)
    {
        HalDashboard.putNumber(name, HalDashboard.getNumber(name, defaultValue));
    }

    public void start()
    {
        targetAngle = HalDashboard.getNumber("Test/TurnDegrees", 0.0);
        startTime = TrcUtil.getCurrentTime();
        robot.globalTracer.traceInfo("MotionMagicTest.start", "Started! Time: %.3f", startTime);
        motionMagic.turn(targetAngle, event);

        if (WRITE_CSV)
        {
            try
            {
                startTime = TrcUtil.getCurrentTime();
                String timeStamp = new SimpleDateFormat("dd-MM-yy_HHmm").format(new Date());
                File dir = new File("/home/lvuser/MotionMagic_logs");
                if (dir.isDirectory() || dir.mkdir())
                {
                    fileOut = new PrintStream(new FileOutputStream(new File(dir, timeStamp + "_profilelog.csv")));
                    fileOut.println("Time,AvgError,LPosition,RPosition,LVelocity,RVelocity");
                }
            }
            catch (IOException e)
            {
                robot.globalTracer.traceErr("MotionMagicTest.start", e.toString());
            }
        }
    }

    public void stop()
    {
        motionMagic.cancel();
    }

    @Override
    public boolean cmdPeriodic(double e)
    {
        double elapsedTime = TrcUtil.getCurrentTime() - startTime;
        if (event.isSignaled() || event.isCanceled())
        {
            double error = targetAngle - robot.driveBase.getYPosition();
            robot.dashboard.displayPrintf(1, "Motion Magic time: %.3f, Error: %.2f", elapsedTime, error);
            if (robot.globalTracer != null)
            {
                if (event.isCanceled())
                {
                    robot.globalTracer
                        .traceInfo("cmdPeriodic", "Motion magic timed out! Time: %.3f, Error:%.2f", elapsedTime, error);
                }
                else
                {
                    robot.globalTracer
                        .traceInfo("cmdPeriodic", "Motion Magic completed! Time: %.3f, Error:%.2f", elapsedTime, error);
                }
            }
            event.clear();
            return true;
        }

        if (motionMagic.isRunning() && fileOut != null)
        {
            fileOut.printf("%.3f,%.2f,%d,%d,%.2f,%.2f\n", elapsedTime, motionMagic.getError(),
                robot.leftFrontWheel.motor.getSelectedSensorPosition(0),
                robot.rightFrontWheel.motor.getSelectedSensorPosition(0),
                robot.leftFrontWheel.getVelocity(),
                robot.rightFrontWheel.getVelocity());
        }
        return false;
    }
}
