package drivesim;

import trclib.TrcPose2D;
import trclib.TrcSwerveDriveBase;
import trclib.TrcSwerveModule;

public class MockSwerveDriveBase extends TrcSwerveDriveBase
{
    private double x, y, heading;
    private double xVel, yVel;

    public MockSwerveDriveBase(TrcSwerveModule leftFrontMotor, TrcSwerveModule leftRearMotor,
        TrcSwerveModule rightFrontMotor, TrcSwerveModule rightRearMotor, double wheelBaseWidth,
        double wheelBaseLength)
    {
        super(leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor, wheelBaseWidth, wheelBaseLength);
        this.setPositionScales(1.0, 1.0, 1.0);
    }

    public void setState(double x, double y, double heading, double xVel, double yVel)
    {
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.xVel = xVel;
        this.yVel = yVel;
    }

    @Override
    protected TrcPose2D updateOdometry(MotorValues motorValues)
    {
        odometry.x = x;
        odometry.y = y;
        odometry.heading = heading;
        odometry.turnRate = heading;
        odometry.xVel = xVel;
        odometry.yVel = yVel;
        return new TrcPose2D();
    }
}
