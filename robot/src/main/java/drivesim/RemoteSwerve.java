package drivesim;

import trclib.TrcDigitalInput;
import trclib.TrcGyro;
import trclib.TrcPidActuator;
import trclib.TrcPidController;
import trclib.TrcRobot;
import trclib.TrcSwerveDriveBase;
import trclib.TrcSwerveModule;
import trclib.TrcTaskMgr;
import trclib.TrcUtil;

import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

public class RemoteSwerve
{
    private final TrcGyro gyro;
    private final TrcSwerveModule lfModule;
    private final TrcSwerveModule rfModule;
    private final TrcSwerveModule lrModule;
    private final TrcSwerveModule rrModule;
    private final TrcSwerveDriveBase driveBase;
    private final TrcTaskMgr taskMgr;
    private Double lastTime;
    private double x, y, heading;

    public RemoteSwerve(double width, double length, TrcGyro gyro)
    {
        TrcTaskMgr.getInstance();

        TrcPidController.PidCoefficients pidCoefficients = new TrcPidController.PidCoefficients(0.02);
        double turnTolerance = 1;

        this.gyro = gyro;

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
        Arrays.stream(new TrcSwerveModule[] { lfModule, rfModule, lrModule, rrModule })
            .forEach(e -> e.setSteeringLimits(-170, 170));

        driveBase = new TrcSwerveDriveBase(lfModule, lrModule, rfModule, rrModule, gyro, width, length);

        taskMgr = TrcTaskMgr.getInstance();
    }

    public SwerveStatus getStatus(double x, double y, double turn)
    {
        taskMgr.executeTaskType(TrcTaskMgr.TaskType.PRECONTINUOUS_TASK, TrcRobot.RunMode.TELEOP_MODE);

        driveBase.holonomicDrive(x, y, turn, gyro.getZHeading().value);

        SwerveStatus status = new SwerveStatus();
        status.lfPower = (float) lfModule.getPower();
        status.rfPower = (float) rfModule.getPower();
        status.lrPower = (float) lrModule.getPower();
        status.rrPower = (float) rrModule.getPower();

        status.lfAngle = (float) lfModule.getSteerAngle();
        status.rfAngle = (float) rfModule.getSteerAngle();
        status.lrAngle = (float) lrModule.getSteerAngle();
        status.rrAngle = (float) rrModule.getSteerAngle();

        taskMgr.executeTaskType(TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK, TrcRobot.RunMode.TELEOP_MODE);

        return status;
    }

    public void updateOdometry(double lfSpeed, double rfSpeed, double lrSpeed, double rrSpeed)
    {
        double[] wheels = new double[] { lfSpeed, rfSpeed, lrSpeed, rrSpeed };
        double[] angles = new double[] { lfModule.getSteerAngle(), rfModule.getSteerAngle(), lrModule.getSteerAngle(),
            rrModule.getSteerAngle() };
        List<double[]> vectors = IntStream.range(0, 4)
            .mapToObj(
                i -> new double[] {
                    wheels[i] * Math.sin(Math.toRadians(angles[i])),
                    wheels[i] * Math.cos(Math.toRadians(angles[i])) })
            .collect(Collectors.toList());

        double vx = vectors.stream().mapToDouble(e -> e[0]).average().orElse(0.0);
        double vy = vectors.stream().mapToDouble(e -> e[1]).average().orElse(0.0);
        double[] lf = vectors.get(0);
        double[] rf = vectors.get(1);
        double[] lr = vectors.get(2);
        double[] rr = vectors.get(3);
        double x = driveBase.getWheelBaseWidth() / driveBase.getWheelBaseDiagonal();
        double y = driveBase.getWheelBaseLength() / driveBase.getWheelBaseDiagonal();
        double omega = x * TrcUtil.average(-rr[0], rf[0], lf[0], -lr[0]) + y * TrcUtil.average(lf[1], -rf[1], lr[1], -rr[1]);
        double currTime = TrcUtil.getCurrentTime();
        if (lastTime != null)
        {
            double dt = currTime - lastTime;
            double rot = Math.toRadians(gyro.getZHeading().value);
            this.x += dt * (vx * Math.cos(rot) + vy * Math.sin(rot));
            this.y += dt * (vy * Math.cos(rot) - vx * Math.sin(rot));
            this.heading += Math.toDegrees(omega) * dt;
            System.out.printf("\rx=%.2f,y=%.2f,rot=%.2f", this.x, this.y, Math.toDegrees(rot));
        }
        lastTime = currTime;
    }

    private static class SwerveStatus
    {
        public float lfPower, lfAngle;
        public float rfPower, rfAngle;
        public float lrPower, lrAngle;
        public float rrPower, rrAngle;
    }
}