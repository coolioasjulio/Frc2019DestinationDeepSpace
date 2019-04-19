package drivesim;

import trclib.TrcDigitalInput;
import trclib.TrcHolonomicPurePursuitController;
import trclib.TrcMotionProfilePoint;
import trclib.TrcPidActuator;
import trclib.TrcPidController;
import trclib.TrcRobot;
import trclib.TrcSwerveModule;
import trclib.TrcTaskMgr;
import trclib.TrcUtil;
import trclib.TrcWarpSpace;

import java.util.Arrays;

public class RemotePP
{
    private final TrcSwerveModule lfModule;
    private final TrcSwerveModule rfModule;
    private final TrcSwerveModule lrModule;
    private final TrcSwerveModule rrModule;
    private final MockSwerveDriveBase driveBase;
    private final TrcTaskMgr taskMgr;
    private volatile double x, y, heading;
    private volatile Double lastTime;
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

        purePursuit = new TrcHolonomicPurePursuitController("PurePursuit", driveBase, 0.5, 0.1,
            new TrcPidController.PidCoefficients(0.1), new TrcPidController.PidCoefficients(0.004, 0.0, 0.0004),
            new TrcPidController.PidCoefficients(0.0, 0.0, 0.0, 1.0 / topSpeed));

        taskMgr = TrcTaskMgr.getInstance();
    }

    public void drive()
    {
        Arrays.stream(PATH).forEach(point -> point.heading = Math.toDegrees(point.heading));
        purePursuit.start(PATH);
    }

    public SwerveStatus getStatus(double x, double y, double heading)
    {
        double xVel = 0;
        double yVel = 0;
        double currTime = TrcUtil.getCurrentTime();
        if (lastTime != null)
        {
            xVel = (x - this.x) / (currTime - lastTime);
            yVel = (y - this.y) / (currTime - lastTime);
        }
        lastTime = currTime;
        this.x = x;
        this.y = y;

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

    private final TrcMotionProfilePoint[] PATH = new TrcMotionProfilePoint[]
        {
            new TrcMotionProfilePoint(0.05,0,0.000733,0.000833,0.033333,0.666667,13.333333,0.000176),
            new TrcMotionProfilePoint(0.05,0.000002,0.004067,0.004167,0.1,1.333333,13.333333,0.000976),
            new TrcMotionProfilePoint(0.05,0.000016,0.011567,0.011667,0.2,2,13.333333,0.002779),
            new TrcMotionProfilePoint(0.05,0.00007,0.024067,0.024167,0.3,2,0,0.005787),
            new TrcMotionProfilePoint(0.05,0.000208,0.041566,0.041667,0.4,2,0,0.010009),
            new TrcMotionProfilePoint(0.05,0.000494,0.064064,0.064167,0.5,2,0,0.015453),
            new TrcMotionProfilePoint(0.05,0.001011,0.091559,0.091667,0.6,2,0,0.022131),
            new TrcMotionProfilePoint(0.05,0.001859,0.124048,0.124167,0.7,2,0,0.030056),
            new TrcMotionProfilePoint(0.05,0.003158,0.161525,0.161667,0.8,2,0,0.039242),
            new TrcMotionProfilePoint(0.05,0.005047,0.203983,0.204167,0.9,2,0,0.049704),
            new TrcMotionProfilePoint(0.05,0.007685,0.25141,0.251667,1,2,0,0.061456),
            new TrcMotionProfilePoint(0.05,0.011251,0.303788,0.304167,1.1,2,0,0.074509),
            new TrcMotionProfilePoint(0.05,0.015943,0.361096,0.361667,1.2,2,0,0.088873),
            new TrcMotionProfilePoint(0.05,0.021978,0.423303,0.424167,1.3,2,0,0.104552),
            new TrcMotionProfilePoint(0.05,0.029592,0.490371,0.491667,1.4,2,0,0.121544),
            new TrcMotionProfilePoint(0.05,0.03904,0.562252,0.564167,1.5,2,0,0.139839),
            new TrcMotionProfilePoint(0.05,0.050592,0.638885,0.641667,1.6,2,0,0.159415),
            new TrcMotionProfilePoint(0.05,0.064536,0.720197,0.724167,1.7,2,0,0.180241),
            new TrcMotionProfilePoint(0.05,0.081169,0.806099,0.811667,1.8,2,0,0.202266),
            new TrcMotionProfilePoint(0.05,0.1008,0.89649,0.904167,1.9,2,0,0.225428),
            new TrcMotionProfilePoint(0.05,0.123744,0.99125,1.001667,2,2,0,0.24964),
            new TrcMotionProfilePoint(0.05,0.150317,1.090243,1.104167,2.1,2,0,0.274801),
            new TrcMotionProfilePoint(0.05,0.180832,1.193317,1.211667,2.2,2,0,0.300784),
            new TrcMotionProfilePoint(0.05,0.215596,1.300308,1.324167,2.3,2,0,0.327445),
            new TrcMotionProfilePoint(0.05,0.254901,1.411035,1.441667,2.4,2,0,0.354619),
            new TrcMotionProfilePoint(0.05,0.299019,1.52531,1.564167,2.5,2,0,0.382124),
            new TrcMotionProfilePoint(0.05,0.348202,1.642938,1.691667,2.6,2,0,0.409763),
            new TrcMotionProfilePoint(0.05,0.40267,1.763721,1.824167,2.7,2,0,0.437327),
            new TrcMotionProfilePoint(0.05,0.462609,1.887464,1.961667,2.8,2,0,0.464603),
            new TrcMotionProfilePoint(0.05,0.528173,2.01398,2.104167,2.9,2,0,0.491373),
            new TrcMotionProfilePoint(0.05,0.599061,2.142373,2.250833,2.966667,1.333333,-13.333333,0.51728),
            new TrcMotionProfilePoint(0.05,0.674433,2.271093,2.4,3,0.666667,-13.333333,0.541882),
            new TrcMotionProfilePoint(0.05,0.753282,2.398693,2.55,3,0,-13.333333,0.564827),
            new TrcMotionProfilePoint(0.05,0.834925,2.524525,2.7,3,0,0,0.585971),
            new TrcMotionProfilePoint(0.05,0.919099,2.648678,2.85,3,0,0,0.605328),
            new TrcMotionProfilePoint(0.05,1.005552,2.771256,3,3,0,0,0.622918),
            new TrcMotionProfilePoint(0.05,1.094042,2.892371,3.15,3,0,0,0.63877),
            new TrcMotionProfilePoint(0.05,1.184339,3.012146,3.3,3,0,0,0.652917),
            new TrcMotionProfilePoint(0.05,1.276222,3.130709,3.45,3,0,0,0.665391),
            new TrcMotionProfilePoint(0.05,1.369481,3.248194,3.6,3,0,0,0.676228),
            new TrcMotionProfilePoint(0.05,1.463913,3.364738,3.75,3,0,0,0.685458),
            new TrcMotionProfilePoint(0.05,1.559326,3.48048,3.9,3,0,0,0.693112),
            new TrcMotionProfilePoint(0.05,1.655532,3.595564,4.05,3,0,0,0.699215),
            new TrcMotionProfilePoint(0.05,1.752351,3.710132,4.2,3,0,0,0.703788),
            new TrcMotionProfilePoint(0.05,1.849607,3.824331,4.35,3,0,0,0.706849),
            new TrcMotionProfilePoint(0.05,1.947126,3.938305,4.5,3,0,0,0.708407),
            new TrcMotionProfilePoint(0.05,2.044738,4.052199,4.65,3,0,0,0.708469),
            new TrcMotionProfilePoint(0.05,2.142271,4.166161,4.8,3,0,0,0.707036),
            new TrcMotionProfilePoint(0.05,2.239555,4.280335,4.95,3,0,0,0.704101),
            new TrcMotionProfilePoint(0.05,2.336418,4.394867,5.1,3,0,0,0.699654),
            new TrcMotionProfilePoint(0.05,2.432682,4.509903,5.25,3,0,0,0.69368),
            new TrcMotionProfilePoint(0.05,2.528168,4.625585,5.4,3,0,0,0.686156),
            new TrcMotionProfilePoint(0.05,2.622689,4.742056,5.55,3,0,0,0.677058),
            new TrcMotionProfilePoint(0.05,2.716053,4.859457,5.7,3,0,0,0.666357),
            new TrcMotionProfilePoint(0.05,2.808059,4.977925,5.85,3,0,0,0.65402),
            new TrcMotionProfilePoint(0.05,2.898496,5.097594,6,3,0,0,0.640014),
            new TrcMotionProfilePoint(0.05,2.987146,5.218593,6.15,3,0,0,0.624306),
            new TrcMotionProfilePoint(0.05,3.073778,5.341045,6.3,3,0,0,0.606861),
            new TrcMotionProfilePoint(0.05,3.158151,5.465062,6.45,3,0,0,0.587653),
            new TrcMotionProfilePoint(0.05,3.240015,5.59075,6.6,3,0,0,0.566658),
            new TrcMotionProfilePoint(0.05,3.319107,5.7182,6.75,3,0,0,0.543864),
            new TrcMotionProfilePoint(0.05,3.394778,5.846824,6.899236,2.969423,-0.611534,-12.230679,0.519399),
            new TrcMotionProfilePoint(0.05,3.466045,5.975244,7.046109,2.905513,-1.278201,-13.333333,0.493597),
            new TrcMotionProfilePoint(0.05,3.532053,6.101918,7.188954,2.80827,-1.944867,-13.333333,0.466894),
            new TrcMotionProfilePoint(0.05,3.59246,6.225893,7.326867,2.70827,-2,-1.102654,0.439655),
            new TrcMotionProfilePoint(0.05,3.647381,6.346925,7.459781,2.60827,-2,0,0.412109),
            new TrcMotionProfilePoint(0.05,3.697,6.464818,7.587694,2.50827,-2,0,0.384471),
            new TrcMotionProfilePoint(0.05,3.741536,6.579375,7.710608,2.40827,-2,0,0.35695),
            new TrcMotionProfilePoint(0.05,3.781236,6.6904,7.828521,2.30827,-2,0,0.329743),
            new TrcMotionProfilePoint(0.05,3.816372,6.797704,7.941435,2.20827,-2,0,0.303034),
            new TrcMotionProfilePoint(0.05,3.847234,6.901107,8.049348,2.10827,-2,0,0.276991),
            new TrcMotionProfilePoint(0.05,3.874128,7.000442,8.152262,2.00827,-2,0,0.251758),
            new TrcMotionProfilePoint(0.05,3.897367,7.095555,8.250175,1.90827,-2,0,0.227464),
            new TrcMotionProfilePoint(0.05,3.917267,7.18631,8.343089,1.80827,-2,0,0.204213),
            new TrcMotionProfilePoint(0.05,3.934142,7.272587,8.431002,1.70827,-2,0,0.18209),
            new TrcMotionProfilePoint(0.05,3.948303,7.354281,8.513916,1.60827,-2,0,0.161164),
            new TrcMotionProfilePoint(0.05,3.960048,7.431303,8.591829,1.50827,-2,0,0.141482),
            new TrcMotionProfilePoint(0.05,3.969664,7.503578,8.664743,1.40827,-2,0,0.123081),
            new TrcMotionProfilePoint(0.05,3.977425,7.571046,8.732656,1.30827,-2,0,0.10598),
            new TrcMotionProfilePoint(0.05,3.983586,7.633657,8.79557,1.20827,-2,0,0.090192),
            new TrcMotionProfilePoint(0.05,3.988384,7.69137,8.853483,1.10827,-2,0,0.07572),
            new TrcMotionProfilePoint(0.05,3.992039,7.744157,8.906396,1.00827,-2,0,0.062558),
            new TrcMotionProfilePoint(0.05,3.994751,7.791994,8.95431,0.90827,-2,0,0.050699),
            new TrcMotionProfilePoint(0.05,3.996699,7.834863,8.997223,0.80827,-2,0,0.04013),
            new TrcMotionProfilePoint(0.05,3.998044,7.872752,9.035137,0.70827,-2,0,0.030838),
            new TrcMotionProfilePoint(0.05,3.998926,7.905654,9.06805,0.60827,-2,0,0.022809),
            new TrcMotionProfilePoint(0.05,3.999468,7.933562,9.095964,0.50827,-2,0,0.016028),
            new TrcMotionProfilePoint(0.05,3.999772,7.956473,9.118877,0.40827,-2,0,0.010482),
            new TrcMotionProfilePoint(0.05,3.999921,7.974386,9.136791,0.30827,-2,0,0.00616),
            new TrcMotionProfilePoint(0.05,3.999981,7.9873,9.149704,0.20827,-2,0,0.003051),
            new TrcMotionProfilePoint(0.05,3.999997,7.995213,9.157618,0.10827,-2,0,0.001149),
            new TrcMotionProfilePoint(0.05,4,7.998891,9.161296,0.038847,-1.388466,12.230679,0.000266),
            new TrcMotionProfilePoint(0.05,4,7.999931,9.162336,0.002757,-0.721799,13.333333,0.000017),
            new TrcMotionProfilePoint(0.05,4,8,9.162405,0,-0.055133,13.333333,0.0),
            new TrcMotionProfilePoint(0.05,4,8,9.162405,0,0,1.102654,0.0)
        };
}
