/*
 * Copyright (c) 2019 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package frclib;

import com.ctre.phoenix.motorcontrol.*;
import trclib.*;

public class FrcMotionMagicRotationController
{
    private TrcPidController.PidCoefficients pidCoefficients;
    private FrcCANTalon rightMaster;
    private TrcEvent onFinishedEvent;
    private TrcTaskMgr.TaskObject motionMagicTaskObj;
    private boolean running = false;
    private boolean cancelled = false;
    private double timeoutTime;
    private boolean autoTimeout = false;
    private double fudgeFactor = 1.0;
    private FrcPigeonIMU pigeon = null;
    private boolean gadgeteer = false;
    private boolean leftInverted;
    private FrcCANTalon[] leftMotors;
    private FrcCANTalon[] rightMotors;
    private TrcWarpSpace warpSpace;
    private double degreesPerUnit = 360.0 / 8192.0;
    /**
     * Units.
     */
    private double errorTolerance;
    /**
     * Units.
     */
    private double targetAngle;
    /**
     * Units per 100ms.
     */
    private int maxRotVelocity;
    /**
     * Units per 100ms per second. Yeah I know it's gross.
     */
    private int maxRotAcceleration;

    /**
     * Creates a motion magic rotation controller.
     *
     * @param instanceName       The name of this instance.
     * @param pidCoefficients    The PIDF coefficients to use for the drivetrain.
     * @param pigeon             The PigeonIMU gyro.
     * @param maxRotVelocity     The maximum speed the robot should go during a move operation.
     *                           The robot may not reach this speed. This should be in world units per second.
     * @param maxRotAcceleration The maximum acceleration of the robot during a move operation.
     *                           The robot may not reach this acceleration. This should be in world units per second per second.
     * @param errorTolerance     The tolerance of error, in world units. If the closed loop error is less than or equal to
     *                           the tolerance, the move operation will be finished.
     * @param gadgeteer          True if connected via a gadgeteer cable to a talonSRX. False if connected to the CAN bus.
     */
    public FrcMotionMagicRotationController(String instanceName, TrcPidController.PidCoefficients pidCoefficients,
        FrcPigeonIMU pigeon, double maxRotVelocity, double maxRotAcceleration, double errorTolerance, boolean gadgeteer)
    {
        this.pidCoefficients = pidCoefficients;
        this.pigeon = pigeon;
        this.gadgeteer = gadgeteer;
        this.errorTolerance = Math.abs(errorTolerance) / degreesPerUnit;
        // Scale velocity and acceleration to encoder units and time frame of 100ms
        this.maxRotVelocity = TrcUtil.round(0.1 * maxRotVelocity / degreesPerUnit);
        // For some reason CTRE is dumb, so acceleration is ticks/100ms/1sec. God I hate these people.
        this.maxRotAcceleration = TrcUtil.round(0.1 * maxRotAcceleration / degreesPerUnit);

        warpSpace = new TrcWarpSpace("MotionMagicWarpSpace", 0.0, 360 / degreesPerUnit);

        this.motionMagicTaskObj = TrcTaskMgr.getInstance().createTask(instanceName, this::motionMagicTask);
    }

    /**
     * Start motion magic move operation.
     *
     * @param targetAngle The target angle to turn to in the domain [0, 360).
     */
    public void turn(double targetAngle)
    {
        turn(targetAngle, null);
    }

    /**
     * Start motion magic move operation.
     *
     * @param targetAngle     The target angle to turn to in the domain [0, 360).
     * @param onFinishedEvent The event to signal when done.
     */
    public void turn(double targetAngle, TrcEvent onFinishedEvent)
    {
        double timeout = autoTimeout ? estimatePathDuration(targetAngle) : Double.POSITIVE_INFINITY;
        turn(targetAngle, onFinishedEvent, timeout);
    }

    /**
     * Start motion magic move operation.
     *
     * @param targetAngle     The target angle to turn to in the domain [0, 360).
     * @param onFinishedEvent The event to signal when done.
     * @param timeout         Number of seconds after which to cancel the movement.
     */
    public void turn(double targetAngle, TrcEvent onFinishedEvent, double timeout)
    {
        if (leftMotors == null || rightMaster == null)
        {
            throw new IllegalStateException("Cannot start before setting both left and right motors!");
        }

        this.timeoutTime = TrcUtil.getCurrentTime() + timeout;

        // Optimize the target angle
        this.targetAngle = warpSpace.getOptimizedTarget(targetAngle / degreesPerUnit, pigeon.getYaw() / degreesPerUnit);

        if (onFinishedEvent != null)
        {
            onFinishedEvent.clear();
        }
        this.onFinishedEvent = onFinishedEvent;

        talonInit();

        // Set the target angle, and the left motors to follow
        for (FrcCANTalon talon : leftMotors)
        {
            talon.motor.follow(rightMaster.motor);
        }
        if (rightMotors.length > 1)
        {
            for (int i = 1; i < rightMotors.length; i++)
            {
                rightMotors[i].motor.follow(rightMaster.motor);
            }
        }
        rightMaster.motor.set(ControlMode.MotionMagic, targetAngle);

        running = true;
        cancelled = false;
        setTaskEnabled(true);
    }

    private void talonInit()
    {
        rightMaster.motor.config_kP(0, pidCoefficients.kP, 0);
        rightMaster.motor.config_kI(0, pidCoefficients.kI, 0);
        rightMaster.motor.config_kD(0, pidCoefficients.kD, 0);
        rightMaster.motor.config_kF(0, pidCoefficients.kF, 0);
        rightMaster.motor.config_IntegralZone(0, pidCoefficients.iZone, 0);

        // Set the left motor inversion to follow that of the right motor, since we will be turning in a circle.
        // This will be cached and reset after the command is done.
        leftInverted = leftMotors[0].getInverted();
        for (FrcCANTalon talon : leftMotors)
        {
            talon.motor.setInverted(rightMaster.motor.getInverted());
        }

        rightMaster.motor.configAllowableClosedloopError(0, TrcUtil.round(errorTolerance), 10);

        // Set the motion magic velocity and acceleration constraints
        rightMaster.motor.configMotionCruiseVelocity(maxRotVelocity, 10);
        rightMaster.motor.configMotionAcceleration(maxRotAcceleration, 10);

        // Add the pigeon IMU as a remote sensor
        rightMaster.motor.configRemoteFeedbackFilter(pigeon.getDeviceID(),
            gadgeteer ? RemoteSensorSource.GadgeteerPigeon_Yaw : RemoteSensorSource.Pigeon_Yaw, 0);

        // Register the gyro as the selected sensor
        rightMaster.motor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0, 0, 10);
    }

    /**
     * Configure auto timeout to be enabled or not. If enabled, unless otherwise specified, automatically compute an
     * upper bound of time required to calculate a timeout value. Fudge factor of 1.1.
     *
     * @param enabled If true, enable auto timeout. Otherwise, disable it.
     */
    public void setAutoTimeoutEnabled(boolean enabled)
    {
        setAutoTimeoutEnabled(enabled, 1.1);
    }

    /**
     * Configure auto timeout to be enabled or not. If enabled, unless otherwise specified, automatically compute an
     * upper bound of time required to calculate a timeout value.
     *
     * @param enabled     If true, enable auto timeout. Otherwise, disable it.
     * @param fudgeFactor Arbitrary value to multiply estimate by to get closer to "real world".
     */
    public void setAutoTimeoutEnabled(boolean enabled, double fudgeFactor)
    {
        autoTimeout = enabled;
        this.fudgeFactor = fudgeFactor;
    }

    /**
     * Sets the error tolerance. If the closed loop error is less than or equal to the tolerance,
     * the move operation will be finished.
     *
     * @param errorTolerance The new error tolerance, in degrees.
     */
    public void setErrorTolerance(double errorTolerance)
    {
        this.errorTolerance = Math.abs(errorTolerance);
    }

    /**
     * Cancel the current move operation.
     */
    public void cancel()
    {
        if (isRunning())
        {
            cancelled = true;
            if (onFinishedEvent != null)
            {
                // Ideally, this should be cancelled instead of signalled, but nobody else ever actually checks the
                // cancelled flag. Whatever.
                onFinishedEvent.set(true);
            }
            stop();
        }
    }

    /**
     * Is there a current move operation in progress?
     *
     * @return True if the motion magic is running, false otherwise.
     */
    public boolean isRunning()
    {
        return running;
    }

    /**
     * Was the last move operation cancelled prematurely?
     *
     * @return True if the last move operation was cancelled, false otherwise.
     */
    public boolean isCancelled()
    {
        return cancelled;
    }

    /**
     * Set the PIDF coefficients. This will only apply for the next time start() is called. It will NOT affect a run
     * already in progress.
     *
     * @param pidCoefficients The PID coefficients to set.
     */
    public void setPidCoefficients(TrcPidController.PidCoefficients pidCoefficients)
    {
        this.pidCoefficients = pidCoefficients;
    }

    /**
     * Sets the motors on the left side of the turn train.
     *
     * @param leftMotors List of motors on the left side of the turn train.
     */
    public void setLeftMotors(FrcCANTalon... leftMotors)
    {
        if (leftMotors.length == 0)
        {
            throw new IllegalArgumentException("Cannot pass empty array of motors!");
        }

        // Store all of them, as they will all need to be used later
        this.leftMotors = leftMotors;
    }

    /**
     * Sets the motors on the right side of the turn train.
     *
     * @param rightMotors List of motors on the right side of the turn train. The first motor in the list will be used
     *                    as the master motor, and all others will be set as slaves.
     */
    public void setRightMotors(FrcCANTalon... rightMotors)
    {
        if (rightMotors.length == 0)
        {
            throw new IllegalArgumentException("Cannot pass empty array of motors!");
        }

        this.rightMaster = rightMotors[0];
        this.rightMotors = rightMotors;
    }

    public double getTargetAngle()
    {
        return rightMaster.motor.getClosedLoopTarget() * degreesPerUnit;
    }

    public double getError()
    {
        return getRawError() * degreesPerUnit;
    }

    private double getRawError()
    {
        return rightMaster.motor.getClosedLoopError();
        //return targetAngle - rightMaster.motor.getSelectedSensorPosition();
    }

    private boolean isDone()
    {
        return running && Math.abs(getRawError()) <= errorTolerance;
    }

    private void stop()
    {
        setTaskEnabled(false);
        onFinishedEvent = null;
        running = false;
        targetAngle = 0.0;
        // Reset the selected feedback device and inversion to the cached value
        rightMaster.motor.configSelectedFeedbackSensor(rightMaster.getFeedbackDevice());
        for (FrcCANTalon talon : leftMotors)
        {
            talon.setInverted(leftInverted);
            talon.motor.set(ControlMode.PercentOutput, 0.0);
        }
        // Stop the motors
        rightMaster.motor.set(ControlMode.PercentOutput, 0.0);
    }

    /**
     * Estimates the duration of the path in seconds. Uses a trapezoidal velocity profile with the provided constraints.
     *
     * @param distance The distance moved in by the path.
     * @return The estimated number of seconds required to move this path. This *should* be an upper bound.
     */
    private double estimatePathDuration(double distance)
    {
        // For convenience, these calculations will be done in world units.
        double maxVelocity = this.maxRotVelocity * 10; // convert to degrees per second
        double maxAcceleration = this.maxRotAcceleration * 10; // convert to deg/sec^2

        // v/t = a
        // vt/2 = d
        // v = max vel, a = max acc, t = time, d = distance
        double rampUpDistance = Math.pow(maxVelocity, 2) / (2.0 * maxAcceleration);
        double estimate;
        if (rampUpDistance * 2 <= distance)
        {
            double cruiseDistance = distance - (rampUpDistance * 2.0);
            double cruiseTime = cruiseDistance / maxVelocity;
            double rampUpTime = maxVelocity / maxAcceleration;
            estimate = rampUpTime * 2 + cruiseTime;
        }
        else
        {
            double halfTime = Math.sqrt(distance / maxAcceleration);
            estimate = 2 * halfTime;
        }
        return estimate * fudgeFactor;
    }

    private void setTaskEnabled(boolean enabled)
    {
        if (enabled)
        {
            motionMagicTaskObj.registerTask(TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK);
        }
        else
        {
            motionMagicTaskObj.unregisterTask(TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK);
        }
    }

    private void motionMagicTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode mode)
    {
        if (isDone())
        {
            TrcDbgTrace.getGlobalTracer().traceInfo("FrcMotionMagicRotationController.task", "Done!");
            if (onFinishedEvent != null)
            {
                onFinishedEvent.set(true);
            }
            stop();
        }
        else if (TrcUtil.getCurrentTime() >= timeoutTime)
        {
            cancel();
        }
    }
}
