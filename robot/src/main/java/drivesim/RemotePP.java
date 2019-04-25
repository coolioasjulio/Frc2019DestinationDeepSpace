package drivesim;

import trclib.TrcDigitalInput;
import trclib.TrcHolonomicPurePursuitController;
import trclib.TrcPath;
import trclib.TrcPidActuator;
import trclib.TrcPidController;
import trclib.TrcRobot;
import trclib.TrcSwerveModule;
import trclib.TrcTaskMgr;
import trclib.TrcUtil;
import trclib.TrcWarpSpace;
import trclib.TrcWaypoint;

import java.util.Arrays;

public class RemotePP
{
    private final TrcSwerveModule lfModule;
    private final TrcSwerveModule rfModule;
    private final TrcSwerveModule lrModule;
    private final TrcSwerveModule rrModule;
    private final MockSwerveDriveBase driveBase;
    private final TrcTaskMgr taskMgr;
    private volatile double heading;
    private TrcWarpSpace warpSpace;
    private TrcHolonomicPurePursuitController purePursuit;

    public RemotePP(double width, double length, double topSpeed)
    {
        TrcTaskMgr.getInstance();

        TrcPidController.PidCoefficients pidCoefficients = new TrcPidController.PidCoefficients(0.02);
        double turnTolerance = 1;

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

        driveBase = new MockSwerveDriveBase(lfModule, lrModule, rfModule, rrModule, width, length);
        driveBase.setOdometryEnabled(true);

        warpSpace = new TrcWarpSpace("warp", 0.0, 360.0);

        purePursuit = new TrcHolonomicPurePursuitController("PurePursuit", driveBase, 1.0, 0.1,
            new TrcPidController.PidCoefficients(0.2), new TrcPidController.PidCoefficients(0.004, 0.0, 0.0004),
            new TrcPidController.PidCoefficients(0.0, 0.0, 0.0, 1.0 / topSpeed));

        taskMgr = TrcTaskMgr.getInstance();
    }

    public void drive()
    {
        purePursuit.start(new TrcPath(true,
            new TrcWaypoint(0.1, 0, 0, 0.0, 0.0, 0, 0, 0),
            new TrcWaypoint(0.1, 0, 1, 0.0, 1.0, 0, 0, 0),
            new TrcWaypoint(0.1, 0, 4, 0.0, 2.5, 0, 0, 0),
            new TrcWaypoint(0.1, 4, 4, 0.0, 2.5, 0, 0, 0),
            new TrcWaypoint(0.1, 4, 7, 0.0, 1.5, 0, 0, 0),
            new TrcWaypoint(0.1, 4, 8, 0.0, 0.0, 0, 0, 0)));
    }

    public SwerveStatus getStatus(double x, double y, double heading, double xVel, double yVel)
    {
        this.heading = warpSpace.getOptimizedTarget(heading, this.heading);
        driveBase.setState(x, y, heading, xVel, yVel);

        Arrays.stream(TrcTaskMgr.TaskType.values()).skip(2).limit(4)
            .forEach(e -> taskMgr.executeTaskType(e, TrcRobot.RunMode.TELEOP_MODE));

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
