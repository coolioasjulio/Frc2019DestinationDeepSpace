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

import edu.wpi.first.networktables.ConnectionNotification;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.EntryNotification;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.Relay;
import trclib.TrcDbgTrace;
import trclib.TrcUtil;

import java.util.LinkedList;
import java.util.List;

public abstract class FrcRemoteVisionProcessor
{
    private final String instanceName;

    private volatile RelativePose relativePose = null;
    protected NetworkTable networkTable;
    private int maxAverageWindow = 10; // the last 10 frames
    private List<RelativePose> frames = new LinkedList<>();
    private final Object framesLock = new Object();
    private Relay ringLight;
    private double timeout;

    public FrcRemoteVisionProcessor(String instanceName, String networkTable, String dataKey)
    {
        this.instanceName = instanceName;
        commonInit(networkTable, dataKey);
    }

    public FrcRemoteVisionProcessor(String instanceName, String networkTable, String dataKey, int relayPort)
    {
        this.instanceName = instanceName;
        commonInit(networkTable, dataKey);
        ringLight = new Relay(relayPort);
        ringLight.setDirection(Relay.Direction.kForward);
    }

    private void commonInit(String networkTableName, String dataKey)
    {
        NetworkTableInstance instance = NetworkTableInstance.getDefault();
        networkTable = instance.getTable(networkTableName);
        instance.addConnectionListener(this::connectionListener, false);
        NetworkTableEntry entry = networkTable.getEntry(dataKey);
        entry.addListener(this::updateTargetInfo,
            EntryListenerFlags.kNew | EntryListenerFlags.kUpdate | EntryListenerFlags.kImmediate);

    }

    public void setFreshnessTimeout(double timeout)
    {
        this.timeout = timeout;
    }

    public void setRingLightEnabled(boolean enabled)
    {
        if (ringLight != null)
        {
            ringLight.set(enabled ? Relay.Value.kOn : Relay.Value.kOff);
        }
    }

    private void connectionListener(ConnectionNotification notification)
    {
        if (!notification.connected)
        {
            TrcDbgTrace.getGlobalTracer()
                .traceInfo("connectionListener", "Client %s disconnected!", notification.conn.remote_ip);
        }
    }

    /**
     * Process the latest data from network tables.
     *
     * @param data The NT value containing the data. The type of the data varies depending on implementation.
     * @return The relative pose of the object being tracked. null if no object detected.
     */
    protected abstract RelativePose processData(NetworkTableValue data);

    private void updateTargetInfo(EntryNotification event)
    {
        RelativePose relativePose = processData(event.value);
        if (relativePose == null)
        {
            // We have not found a pose, so set to null
            this.relativePose = null;
            synchronized (framesLock)
            {
                // Clear the frames list if it is not empty
                if (!frames.isEmpty())
                {
                    frames.clear();
                }
            }
        }
        else
        {
            // Deserialize the latest calculated pose
            this.relativePose = relativePose;
            synchronized (framesLock)
            {
                // Add the latest pose
                frames.add(relativePose);
                // Trim the list so only the last few are kept
                while (frames.size() > maxAverageWindow)
                {
                    frames.remove(0);
                }
            }
        }
    }

    private boolean isFresh(RelativePose pose)
    {
        return pose != null && (timeout == 0.0 || TrcUtil.getCurrentTime() - pose.time <= timeout);
    }

    /**
     * Get the average pose of the last n frames where n=maxAverageWindow.
     *
     * @return The average pose. (all attributes averaged)
     */
    public RelativePose getAveragePose()
    {
        return getAveragePose(maxAverageWindow);
    }

    /**
     * Calculates the average pose of the last numFrames frames.
     *
     * @param numFrames How many frames to average.
     * @return The average pose.
     */
    public RelativePose getAveragePose(int numFrames)
    {
        return getAveragePose(numFrames, false);
    }

    /**
     * Calculates the average pose of the last numFrames frames, optionally requiring numFrames frames.
     *
     * @param numFrames  How many frames to average.
     * @param requireAll If true, require at least numFrames frames to average.
     * @return Average of last numFrames frames, or null if not enough frames and requireAll is true.
     */
    public RelativePose getAveragePose(int numFrames, boolean requireAll)
    {
        RelativePose average = new RelativePose();
        synchronized (framesLock)
        {
            if ((requireAll && frames.size() < numFrames) || frames.isEmpty())
            {
                return null;
            }
            int fromIndex = Math.max(0, frames.size() - numFrames);
            List<RelativePose> poses = frames.subList(fromIndex, frames.size());
            int numFreshFrames = 0;
            for (RelativePose pose : poses)
            {
                // Only use data if it's fresh
                if (isFresh(pose))
                {
                    average.objectYaw += pose.objectYaw;
                    average.r += pose.r;
                    average.theta += pose.theta;
                    average.x += pose.x;
                    average.y += pose.y;
                    numFreshFrames++;
                }
            }
            // If no fresh data, clear all and return null
            if (numFreshFrames == 0)
            {
                frames.clear();
                return null;
            }
            average.objectYaw /= (double) numFreshFrames;
            average.r /= (double) numFreshFrames;
            average.theta /= (double) numFreshFrames;
            average.x /= (double) numFreshFrames;
            average.y /= (double) numFreshFrames;
        }
        return average;
    }

    /**
     * Set the max number of frames to keep. You will not be able to average more frames than this.
     *
     * @param numFrames How many frames to keep for averaging at maximum.
     */
    public void setMaxAverageWindow(int numFrames)
    {
        if (numFrames < 0)
        {
            throw new IllegalArgumentException("numFrames must be >= 0!");
        }
        this.maxAverageWindow = numFrames;
    }

    /**
     * Gets the last calculated pose.
     *
     * @return The pose calculated by the vision system. If no object detected, returns null.
     */
    public RelativePose getLastPose()
    {
        RelativePose pose = relativePose;
        return isFresh(pose) ? pose : null;
    }

    public static class RelativePose
    {
        public double r, theta, objectYaw, x, y;
        public double time;
    }
}