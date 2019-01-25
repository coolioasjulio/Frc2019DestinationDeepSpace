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

package team492;

import frclib.FrcCANSparkMax;
import trclib.TrcPidActuator;
import trclib.TrcPidController;
import trclib.TrcUtil;

public class Elevator
{
    private TrcPidActuator elevator;

    public Elevator()
    {
        // TODO: Make sure we're actually using a spark max. Also, check the elevator can id.
        FrcCANSparkMax motor = new FrcCANSparkMax("ElevatorMotor", RobotInfo.CANID_ELEVATOR, true);
        motor.configFwdLimitSwitchNormallyOpen(false);
        motor.configRevLimitSwitchNormallyOpen(false);
        motor.setInverted(true);
        motor.setBrakeModeEnabled(true);

        // TODO: Tune ALL of these constants
        TrcPidController.PidCoefficients pidCoefficients = new TrcPidController.PidCoefficients(RobotInfo.ELEVATOR_KP,
            RobotInfo.ELEVATOR_KI, RobotInfo.ELEVATOR_KD);
        TrcPidController pidController = new TrcPidController("ElevatorPidController", pidCoefficients,
            RobotInfo.ELEVATOR_TOLERANCE, this::getPosition);
        elevator = new TrcPidActuator("ElevatorActuator", motor, pidController, RobotInfo.ELEVATOR_CALIBRATE_POWER,
            RobotInfo.ELEVATOR_PID_FLOOR, RobotInfo.ELEVATOR_PID_CEILING, () -> RobotInfo.ELEVATOR_GRAVITY_COMP);
        elevator.setPositionScale(RobotInfo.ELEVATOR_INCHES_PER_COUNT, RobotInfo.ELEVATOR_MIN_POS);
        elevator.setStallProtection(RobotInfo.ELEVATOR_STALL_MIN_POWER, RobotInfo.ELEVATOR_STALL_TIMEOUT,
            RobotInfo.ELEVATOR_STALL_RESET_TIMEOUT);
    }

    public void setPosition(double position)
    {
        position = TrcUtil.clipRange(position, RobotInfo.ELEVATOR_MIN_POS, RobotInfo.ELEVATOR_MAX_POS);
        elevator.setTarget(position, position != RobotInfo.ELEVATOR_MIN_POS);
    }

    public double getPosition()
    {
        return elevator.getPosition();
    }

    public void setPower(double power)
    {
        elevator.setPower(power, true);
    }
}