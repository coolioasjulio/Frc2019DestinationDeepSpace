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

import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;
import trclib.TrcPidController;
import trclib.TrcServo;

public class FrcSparkSmartMotionServo extends TrcServo
{
    private FrcCANSparkMax spark;
    private CANPIDController pidController;

    public FrcSparkSmartMotionServo(String instanceName, FrcCANSparkMax spark,
        TrcPidController.PidCoefficients pidCoefficients, double tolerance, double maxVelocity,
        double maxAcceleration)
    {
        super(instanceName);

        this.spark = spark;
        this.pidController = spark.motor.getPIDController();

        pidController.setSmartMotionAccelStrategy(CANPIDController.AccelStrategy.kTrapezoidal, 0);
        pidController.setSmartMotionMinOutputVelocity(0.0, 0);
        setPidCoefficients(pidCoefficients);
        setMotionConstraints(maxVelocity, maxAcceleration);
        setOutputRange(-1.0, 1.0);
        setTolerance(tolerance);
    }

    public void setTolerance(double tolerance)
    {
        pidController.setSmartMotionAllowedClosedLoopError(tolerance, 0);
    }

    public void setMotionConstraints(double maxVelocity, double maxAcceleration)
    {
        pidController.setSmartMotionMaxAccel(maxAcceleration, 0);
        pidController.setSmartMotionMaxVelocity(maxVelocity, 0);
    }

    public void setOutputRange(double minOutput, double maxOutput)
    {
        pidController.setOutputRange(minOutput, maxOutput);
    }

    public void setPidCoefficients(TrcPidController.PidCoefficients pidCoefficients)
    {
        pidController.setP(pidCoefficients.kP);
        pidController.setI(pidCoefficients.kI);
        pidController.setD(pidCoefficients.kD);
        pidController.setFF(pidCoefficients.kF);
    }

    @Override
    public void setInverted(boolean inverted)
    {
        spark.setInverted(inverted);
    }

    @Override
    public boolean isInverted()
    {
        return spark.getInverted();
    }

    @Override
    public void setPosition(double position)
    {
        pidController.setReference(position, ControlType.kSmartMotion);
    }

    @Override
    public double getPosition()
    {
        return spark.getPosition();
    }
}
