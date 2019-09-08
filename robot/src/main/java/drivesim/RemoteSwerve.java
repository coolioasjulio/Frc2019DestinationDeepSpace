package drivesim;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import trclib.TrcDigitalInput;
import trclib.TrcGyro;
import trclib.TrcPidActuator;
import trclib.TrcPidController;
import trclib.TrcPose2D;
import trclib.TrcRobot;
import trclib.TrcSwerveDriveBase;
import trclib.TrcSwerveModule;
import trclib.TrcTaskMgr;
import trclib.TrcUtil;

import java.util.stream.IntStream;

public class RemoteSwerve
{
    private static final double DRIVE_SPEED = 3600;

    private final TrcGyro gyro;
    private final TrcSwerveModule lfModule;
    private final TrcSwerveModule rfModule;
    private final TrcSwerveModule lrModule;
    private final TrcSwerveModule rrModule;
    private SimulatedMotorController lfDrive, rfDrive, lrDrive, rrDrive;
    private final TrcSwerveDriveBase driveBase;
    private final TrcTaskMgr taskMgr;
    private Double lastTime;
    private TrcPose2D odometry;
    private final RealVector[] velocities;

    public RemoteSwerve(double width, double length, TrcGyro gyro)
    {
        TrcTaskMgr.getInstance();

        TrcPidController.PidCoefficients pidCoefficients = new TrcPidController.PidCoefficients(0.02);
        double turnTolerance = 1;

        this.gyro = gyro;

        velocities = IntStream.range(0, 4).mapToObj(e -> new ArrayRealVector(new double[] { 0, 0 }))
            .toArray(RealVector[]::new);
        odometry = new TrcPose2D();

        SimulatedMotorController lfMotor = new SimulatedMotorController(1080);
        SimulatedMotorController rfMotor = new SimulatedMotorController(1080);
        SimulatedMotorController lrMotor = new SimulatedMotorController(1080);
        SimulatedMotorController rrMotor = new SimulatedMotorController(1080);

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

        lfDrive = new SimulatedMotorController(3600);
        rfDrive = new SimulatedMotorController(3600);
        lrDrive = new SimulatedMotorController(3600);
        rrDrive = new SimulatedMotorController(3600);

        lfModule = new TrcSwerveModule("lfModule", lfDrive, lfActuator);
        rfModule = new TrcSwerveModule("lfModule", rfDrive, rfActuator);
        lrModule = new TrcSwerveModule("lfModule", lrDrive, lrActuator);
        rrModule = new TrcSwerveModule("lfModule", rrDrive, rrActuator);

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

        try
        {
            status.lfAngle = TrcSwerveModule.class.getDeclaredField("prevSteerAngle").getFloat(lfModule);
            status.rfAngle = TrcSwerveModule.class.getDeclaredField("prevSteerAngle").getFloat(rfModule);
            status.lrAngle = TrcSwerveModule.class.getDeclaredField("prevSteerAngle").getFloat(lrModule);
            status.rrAngle = TrcSwerveModule.class.getDeclaredField("prevSteerAngle").getFloat(rrModule);
        }
        catch (Exception e)
        {
            e.printStackTrace();
        }

        taskMgr.executeTaskType(TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK, TrcRobot.RunMode.TELEOP_MODE);

        return status;
    }

    public void updateOdometry(double lfSpeed, double rfSpeed, double lrSpeed, double rrSpeed)
    {
        double currTime = TrcUtil.getCurrentTime();
        if (lastTime == null)
        {
            lastTime = currTime;
            return;
        }
        double dt = currTime - lastTime;
        lastTime = currTime;

        TrcSwerveModule[] modules = new TrcSwerveModule[] { lfModule, rfModule, lrModule, rrModule };
        RealVector[] wheelVectors = new RealVector[4];
        double[] wheelVels = new double[] { lfSpeed, rfSpeed, lrSpeed, rrSpeed };
        for (int i = 0; i < modules.length; i++)
        {
            double angle = modules[i].getSteerAngle();
            RealVector vel = TrcUtil.polarToCartesian(wheelVels[i], angle);
            wheelVectors[i] = vel.add(velocities[i]).mapMultiply(0.5).mapMultiply(dt);
            velocities[i] = vel;
        }

        RealVector posSum = new ArrayRealVector(2);
        RealVector velSum = new ArrayRealVector(2);
        for (int i = 0; i < 4; i++)
        {
            posSum = posSum.add(wheelVectors[i]);
            velSum = velSum.add(velocities[i]);
        }

        posSum.mapMultiplyToSelf(0.25);
        velSum.mapMultiplyToSelf(0.25);

        TrcPose2D odometry = new TrcPose2D();

        odometry.x = posSum.getEntry(0);
        odometry.y = posSum.getEntry(1);

        odometry.xVel = velSum.getEntry(0);
        odometry.yVel = velSum.getEntry(1);

        double wheelBaseWidth = 1;
        double wheelBaseLength = 1;
        double wheelBaseDiagonal = TrcUtil.magnitude(wheelBaseLength, wheelBaseWidth);
        try
        {
            wheelBaseWidth = TrcSwerveDriveBase.class.getDeclaredField("wheelBaseWidth").getDouble(driveBase);
            wheelBaseLength = TrcSwerveDriveBase.class.getDeclaredField("wheelBaseLength").getDouble(driveBase);
            wheelBaseDiagonal = TrcSwerveDriveBase.class.getDeclaredField("wheelBaseDiagonal").getDouble(driveBase);
        }
        catch (Exception e)
        {
            e.printStackTrace();
        }

        double x = wheelBaseWidth / 2;
        double y = wheelBaseLength / 2;
        // This is black magic math, and it actually needs to be tested.
        double dRot = x * (wheelVectors[0].getEntry(1) + wheelVectors[2].getEntry(1) - wheelVectors[1].getEntry(1)
            - wheelVectors[3].getEntry(1)) + y * (wheelVectors[0].getEntry(0) + wheelVectors[1].getEntry(0)
            - wheelVectors[2].getEntry(0) - wheelVectors[3].getEntry(0));
        dRot /= 4 * Math.pow(wheelBaseDiagonal, 2);
        dRot = Math.toDegrees(dRot);
        odometry.heading = dRot;

        double rotVel =
            x * (velocities[0].getEntry(1) + velocities[2].getEntry(1) - velocities[1].getEntry(1) - velocities[3]
                .getEntry(1)) + y * (velocities[0].getEntry(0) + velocities[1].getEntry(0) - velocities[2].getEntry(0)
                - velocities[3].getEntry(0));
        rotVel /= 4 * Math.pow(wheelBaseDiagonal, 2);
        rotVel = Math.toDegrees(rotVel);
        odometry.turnRate = rotVel;

        double heading = gyro.getZHeading().value;
        odometry.heading = heading - this.odometry.heading;

        updatePose(odometry, this.odometry.heading);
        System.out.printf("\rx=%.2f,y=%.2f,heading=%.2f", this.odometry.x, this.odometry.y, this.odometry.heading);
    }

    private void updatePose(TrcPose2D poseDelta, double heading)
    {
        // The math below uses a different coordinate system (NWU) so we have to convert
        RealMatrix changeOfBasis = MatrixUtils.createRealMatrix(new double[][] { { 0, 1 }, { -1, 0 } });
        double[] posArr = changeOfBasis.operate(new double[] { poseDelta.x, poseDelta.y });
        double x = posArr[0];
        double y = posArr[1];
        // Convert clockwise degrees to counter-clockwise radians
        double theta = Math.toRadians(-poseDelta.heading);
        double headingRad = Math.toRadians(-heading);

        // The following black magic has been ripped straight out of some book (https://file.tavsys.net/control/state-space-guide.pdf)
        RealMatrix A = MatrixUtils.createRealMatrix(new double[][] { { Math.cos(headingRad), -Math.sin(headingRad), 0 },
            { Math.sin(headingRad), Math.cos(headingRad), 0 }, { 0, 0, 1 } });
        RealMatrix B;
        if (Math.abs(theta) <= 1E-9)
        {
            // Use the taylor series approximations, since some values are indeterminate
            B = MatrixUtils.createRealMatrix(new double[][] { { 1 - theta * theta / 6.0, -theta / 2.0, 0 },
                { theta / 2.0, 1 - theta * theta / 6.0, 0 }, { 0, 0, 1 } });
        }
        else
        {
            B = MatrixUtils.createRealMatrix(new double[][] { { Math.sin(theta), Math.cos(theta) - 1, 0 },
                { 1 - Math.cos(theta), Math.sin(theta), 0 }, { 0, 0, theta } });
            B = B.scalarMultiply(1.0 / theta);
        }
        RealVector C = MatrixUtils.createRealVector(new double[] { x, y, theta });
        RealVector globalPose = A.multiply(B).operate(C);
        // Convert back to our (ENU) reference frame
        RealVector pos = changeOfBasis.transpose().operate(globalPose.getSubVector(0, 2));
        // Convert back to clockwise degrees for heading
        theta = Math.toDegrees(-globalPose.getEntry(2));

        // Rotate the velocity vector into the global reference frame
        RealVector vel = MatrixUtils.createRealVector(new double[] { poseDelta.xVel, poseDelta.yVel });
        vel = TrcUtil.rotateCW(vel, heading);

        // Update the odometry values
        odometry.x += pos.getEntry(0);
        odometry.y += pos.getEntry(1);
        odometry.xVel = vel.getEntry(0);
        odometry.yVel = vel.getEntry(1);
        odometry.heading += theta;
        odometry.turnRate = poseDelta.turnRate;
    }

    private static class SwerveStatus
    {
        public float lfPower, lfAngle;
        public float rfPower, rfAngle;
        public float lrPower, lrAngle;
        public float rrPower, rrAngle;
    }
}