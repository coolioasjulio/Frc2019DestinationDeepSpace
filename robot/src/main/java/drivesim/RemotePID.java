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

        SimulatedMotorController lfMotor = new SimulatedMotorController(360);
        SimulatedMotorController rfMotor = new SimulatedMotorController(360);
        SimulatedMotorController lrMotor = new SimulatedMotorController(360);
        SimulatedMotorController rrMotor = new SimulatedMotorController(360);

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

        TrcPidController xPid = new TrcPidController("xpid", new TrcPidController.PidCoefficients(0.1), 0.2, () -> x);
        TrcPidController yPid = new TrcPidController("ypid", new TrcPidController.PidCoefficients(0.1), 0.2, () -> y);
        TrcPidController turnPid = new TrcPidController("turnpid", new TrcPidController.PidCoefficients(0.004, 0.0, 0.0004), 2, () -> heading);
        turnPid.setAbsoluteSetPoint(true);

        pidDrive = new TrcPidDrive("PID", driveBase, xPid, yPid, turnPid);
        pidDrive.setWarpSpaceEnabled(true);

        taskMgr = TrcTaskMgr.getInstance();
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

        Arrays.stream(TrcTaskMgr.TaskType.values()).skip(2).limit(4).forEach(e -> taskMgr.executeTaskType(e, TrcRobot.RunMode.TELEOP_MODE));

        SwerveStatus status = new SwerveStatus();
        status.lfPower = (float) lfModule.getPower();
        status.rfPower = (float) rfModule.getPower();
        status.lrPower = (float) lrModule.getPower();
        status.rrPower = (float) rrModule.getPower();

        status.lfAngle = (float) lfModule.getSteerAngle();
        status.rfAngle = (float) rfModule.getSteerAngle();
        status.lrAngle = (float) lrModule.getSteerAngle();
        status.rrAngle = (float) rrModule.getSteerAngle();

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