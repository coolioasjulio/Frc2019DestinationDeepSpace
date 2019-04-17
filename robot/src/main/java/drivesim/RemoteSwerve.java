package drivesim;

import com.coolioasjulio.rpc.server.RPCServer;
import trclib.TrcDigitalInput;
import trclib.TrcGyro;
import trclib.TrcPidActuator;
import trclib.TrcPidController;
import trclib.TrcRobot;
import trclib.TrcSwerveDriveBase;
import trclib.TrcSwerveModule;
import trclib.TrcTaskMgr;
import trclib.TrcUtil;

import java.io.IOException;
import java.net.ServerSocket;
import java.net.Socket;

public class RemoteSwerve
{
    public static void main(String[] args)
    {
        try {
            ServerSocket serverSocket = new ServerSocket(4444);
            System.out.println("Waiting for connection...");
            Socket socket = serverSocket.accept();
            System.out.println("Received connection from " + socket.getInetAddress().toString());

            RPCServer.getInstance().createRPCSession(socket.getInputStream(), socket.getOutputStream());
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private final TrcGyro gyro;
    private final TrcSwerveModule lfModule;
    private final TrcSwerveModule rfModule;
    private final TrcSwerveModule lrModule;
    private final TrcSwerveModule rrModule;
    private final TrcSwerveDriveBase driveBase;
    private final TrcTaskMgr taskMgr;
    private final SwerveModel model;
    private double xPos, yPos, heading;
    private Double lastTime;

    public RemoteSwerve(double width, double length, TrcGyro gyro)
    {
        TrcTaskMgr.getInstance();

        TrcPidController.PidCoefficients pidCoefficients = new TrcPidController.PidCoefficients(0.02);
        double turnTolerance = 1;

        this.gyro = gyro;
        model = new SwerveModel(length, width, gyro);

        lastTime = null;
        xPos = 0.0;
        yPos = 0.0;
        heading = 0.0;

        MockMotorController lfMotor = new MockMotorController(360);
        MockMotorController rfMotor = new MockMotorController(360);
        MockMotorController lrMotor = new MockMotorController(360);
        MockMotorController rrMotor = new MockMotorController(360);

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

        lfModule = new TrcSwerveModule("lfModule", new MockMotorController(3600), lfActuator);
        rfModule = new TrcSwerveModule("lfModule", new MockMotorController(3600), rfActuator);
        lrModule = new TrcSwerveModule("lfModule", new MockMotorController(3600), lrActuator);
        rrModule = new TrcSwerveModule("lfModule", new MockMotorController(3600), rrActuator);

        driveBase = new TrcSwerveDriveBase(lfModule, lrModule, rfModule, rrModule, gyro, width, length);

        taskMgr = TrcTaskMgr.getInstance();
    }

    public SwerveStatus getStatus(double x, double y, double turn, double gyroAngle, double lfSpeed, double rfSpeed,
        double lrSpeed, double rrSpeed)
    {
        taskMgr.executeTaskType(TrcTaskMgr.TaskType.PRECONTINUOUS_TASK, TrcRobot.RunMode.TELEOP_MODE);

        driveBase.holonomicDrive(x, y, turn, gyroAngle);

        SwerveStatus status = new SwerveStatus();
        status.lfPower = (float) lfModule.getPower();
        status.rfPower = (float) rfModule.getPower();
        status.lrPower = (float) lrModule.getPower();
        status.rrPower = (float) rrModule.getPower();

        status.lfAngle = (float) lfModule.getSteerAngle();
        status.rfAngle = (float) rfModule.getSteerAngle();
        status.lrAngle = (float) lrModule.getSteerAngle();
        status.rrAngle = (float) rrModule.getSteerAngle();

        if (lastTime != null)
        {
            double[] robotVelocity = model
                .getRobotVelocity(status.lfAngle, status.rfAngle, status.lrAngle, status.rrAngle, lfSpeed, rfSpeed, lrSpeed,
                    rrSpeed);

            double deltaTime = TrcUtil.getCurrentTime() - lastTime;
            double gyroRadians = Math.toRadians(gyroAngle);
            yPos += (robotVelocity[1] * Math.cos(gyroRadians) + robotVelocity[0] * Math.sin(gyroRadians)) * deltaTime;
            xPos += (-robotVelocity[1] * Math.sin(gyroRadians) + robotVelocity[0] * Math.cos(gyroRadians)) * deltaTime;
            heading += Math.toDegrees(robotVelocity[2]) * deltaTime;
            status.x = (float) xPos;
            status.y = (float) yPos;
            status.heading = (float) heading;
        }
        lastTime = TrcUtil.getCurrentTime();

        taskMgr.executeTaskType(TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK, TrcRobot.RunMode.TELEOP_MODE);

        return status;
    }

    private static class SwerveStatus
    {
        public float lfPower, lfAngle;
        public float rfPower, rfAngle;
        public float lrPower, lrAngle;
        public float rrPower, rrAngle;
        public float x, y, heading;
    }
}