package drivesim;

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
    protected void updateOdometry(Odometry odometry)
    {
        odometry.xRawPos = x;
        odometry.yRawPos = y;
        odometry.gyroHeading = heading;
        odometry.rotRawPos = heading;
        odometry.xRawVel = xVel;
        odometry.yRawVel = yVel;
    }
}
