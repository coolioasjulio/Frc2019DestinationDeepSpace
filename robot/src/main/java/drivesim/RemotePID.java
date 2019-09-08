package drivesim;

import trclib.TrcDigitalInput;
import trclib.TrcGyro;
import trclib.TrcPidActuator;
import trclib.TrcPidController;
import trclib.TrcPidDrive;
import trclib.TrcRobot;
import trclib.TrcSwerveDriveBase;
import trclib.TrcSwerveModule;
import trclib.TrcTaskMgr;
import trclib.TrcWarpSpace;

import java.lang.reflect.Field;
import java.util.Arrays;

public class RemotePID
{
    private final TrcSwerveModule lfModule;
    private final TrcSwerveModule rfModule;
    private final TrcSwerveModule lrModule;
    private final TrcSwerveModule rrModule;
    private final TrcSwerveDriveBase driveBase;
    private final TrcPidDrive pidDrive;
    private final TrcTaskMgr taskMgr;
    private volatile double x, y, heading;
    private TrcWarpSpace warpSpace;

    public RemotePID(double width, double length)
    {
        TrcTaskMgr.getInstance();

        TrcPidController.PidCoefficients pidCoefficients = new TrcPidController.PidCoefficients(0.02);
        double turnTolerance = 1;

        TrcGyro gyro = new MockGyro("gyro");

        SimulatedMotorController lfMotor = new SimulatedMotorController(720);
        SimulatedMotorController rfMotor = new SimulatedMotorController(720);
        SimulatedMotorController lrMotor = new SimulatedMotorController(720);
        SimulatedMotorController rrMotor = new SimulatedMotorController(720);

        TrcPidController lfCtrl = new TrcPidController("LFPID", pidCoefficients, turnTolerance, lfMotor::getPosition);
        TrcPidController rfCtrl = new TrcPidController("RFPID", pidCoefficients, turnTolerance, rfMotor::getPosition);
        TrcPidController lrCtrl = new TrcPidController("LRPID", pidCoefficients, turnTolerance, lrMotor::getPosition);
        TrcPidController rrCtrl = new TrcPidController("RRPID", pidCoefficients, turnTolerance, rrMotor::getPosition);

        TrcDigitalInput in = new TrcDigitalInput("Input")
        {
            @Override
            public boolean isActive()
            {
                return false;
            }
        };

        TrcPidActuator lfActuator = new TrcPidActuator("LF_ACT", lfMotor, in, lfCtrl, 0.1);
        TrcPidActuator rfActuator = new TrcPidActuator("RF_ACT", rfMotor, in, rfCtrl, 0.1);
        TrcPidActuator lrActuator = new TrcPidActuator("LR_ACT", lrMotor, in, lrCtrl, 0.1);
        TrcPidActuator rrActuator = new TrcPidActuator("RR_ACT", rrMotor, in, rrCtrl, 0.1);

        lfModule = new TrcSwerveModule("lfModule", new SimulatedMotorController(3600), lfActuator);
        rfModule = new TrcSwerveModule("lfModule", new SimulatedMotorController(3600), rfActuator);
        lrModule = new TrcSwerveModule("lfModule", new SimulatedMotorController(3600), lrActuator);
        rrModule = new TrcSwerveModule("lfModule", new SimulatedMotorController(3600), rrActuator);

        driveBase = new TrcSwerveDriveBase(lfModule, lrModule, rfModule, rrModule, gyro, width, length);

        warpSpace = new TrcWarpSpace("warp", 0.0, 360.0);

        TrcPidController xPid = new TrcPidController("xpid", new TrcPidController.PidCoefficients(0.15), 0.1, () -> x);
        TrcPidController yPid = new TrcPidController("ypid", new TrcPidController.PidCoefficients(0.15), 0.1, () -> y);
        TrcPidController turnPid = new TrcPidController("turnpid", new TrcPidController.PidCoefficients(0.005, 0.0, 0.0004), 2, () -> heading);
        turnPid.setAbsoluteSetPoint(true);

        pidDrive = new TrcPidDrive("PID", driveBase, xPid, yPid, turnPid);
        pidDrive.setWarpSpaceEnabled(true);

        taskMgr = TrcTaskMgr.getInstance();
    }

    private float getTargetAngle(TrcSwerveModule module)
    {
        try
        {
            Field field = module.getClass().getDeclaredField("prevSteerAngle");
            field.setAccessible(true);
            return (float) field.getDouble(module);
        }
        catch (NoSuchFieldException | IllegalAccessException e)
        {
            e.printStackTrace();
            return 0;
        }
    }

    public void drive(double x, double y, double heading)
    {
        pidDrive.setTarget(x, y, heading, false, null);
    }

    public SwerveStatus getStatus(double x, double y, double heading)
    {
        this.x = x;
        this.y = y;
        this.heading = warpSpace.getOptimizedTarget(heading, this.heading);

        Arrays.stream(TrcTaskMgr.TaskType.values()).skip(2).limit(4)
            .forEach(e -> taskMgr.executeTaskType(e, TrcRobot.RunMode.TELEOP_MODE));

        SwerveStatus status = new SwerveStatus();
        status.lfPower = (float) lfModule.getPower();
        status.rfPower = (float) rfModule.getPower();
        status.lrPower = (float) lrModule.getPower();
        status.rrPower = (float) rrModule.getPower();

        try
        {
            status.lfAngle = getTargetAngle(lfModule);
            status.rfAngle = getTargetAngle(rfModule);
            status.lrAngle = getTargetAngle(lrModule);
            status.rrAngle = getTargetAngle(rrModule);
        }
        catch (Exception e)
        {
            e.printStackTrace();
        }

        return status;
    }

    private static class SwerveStatus
    {
        public float lfPower, lfAngle;
        public float rfPower, rfAngle;
        public float lrPower, lrAngle;
        public float rrPower, rrAngle;
    }
}
