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

import com.ctre.phoenix.motorcontrol.ControlMode;
import trclib.TrcPidController;
import trclib.TrcServo;
import trclib.TrcUtil;
import trclib.TrcWarpSpace;

public class FrcTalonServo extends TrcServo
{
    private FrcCANTalon talon;
    private TrcWarpSpace warpSpace;
    private double degreesPerTick;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name of the servo.
     * @param talon the physical talon motor controller object.
     * @param pidCoefficients the pid coefficients used for motion magic. Don't forget kF!
     * @param degreesPerTick degrees per native sensor unit measured by the talon.
     * @param maxSpeed desired max speed of the motor, in degrees per second.
     * @param maxAccel desired max acceleration of the motor, in degrees per second per second.
     * @param continuousRotation if true, servo can rotate indefinitely. If false, there is a physical hard limit.
     */
    public FrcTalonServo(String instanceName, FrcCANTalon talon, TrcPidController.PidCoefficients pidCoefficients,
        double degreesPerTick, double maxSpeed, double maxAccel, boolean continuousRotation)
    {
        super(instanceName);
        this.talon = talon;
        if (continuousRotation)
        {
            warpSpace = new TrcWarpSpace(instanceName + ".warpSpace", 0.0, 360.0);
        }
        this.degreesPerTick = degreesPerTick;

        talon.motor.config_kP(0, pidCoefficients.kP);
        talon.motor.config_kI(0, pidCoefficients.kI);
        talon.motor.config_kD(0, pidCoefficients.kD);
        talon.motor.config_kF(0, pidCoefficients.kF);
        talon.motor.configMotionCruiseVelocity(TrcUtil.round((maxSpeed / degreesPerTick) / 10));
        talon.motor.configMotionAcceleration(TrcUtil.round((maxAccel / degreesPerTick) / 10));
    }

    @Override
    public void setInverted(boolean inverted)
    {
        talon.setInverted(inverted);
    }

    @Override
    public boolean isInverted()
    {
        return talon.getInverted();
    }

    @Override
    public void setPosition(double position)
    {
        double angle = position * 360.0;
        if (warpSpace != null)
        {
            angle = warpSpace.getOptimizedTarget(angle, getPosition());
        }
        int ticks = TrcUtil.round(angle / degreesPerTick);
        talon.motor.set(ControlMode.MotionMagic, ticks);
    }

    @Override
    public double getPosition()
    {
        return talon.getPosition() * degreesPerTick;
    }
}
