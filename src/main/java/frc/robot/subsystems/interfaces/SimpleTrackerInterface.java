/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.interfaces;

/**
 * This interface is implemented by subsystems that provide the image location
 * of a set of tracked objects to clients.
 */
public interface SimpleTrackerInterface {

    /**
     * This is the data for each active target in the image.
     */
    public static class TargetData {
        public int cx = 0;  // X coordinate of target center in image (pixels)
        public int cy = 0;  // Y coordinate of target center in image (pixels)
        public int vx = 0;  // X veloicity of target in image pixels/second
        public int vy = 0;  // Y velocity of target in image pixels/second
        public int targetType = 0; // Target type: 0-15 allows different targets
        public int quality = 0; // Quality of target data in percent. 0 = lost/empty, 100 = perfect lock.
        public long timestamp = 0; // Timestamp of this target data update.

        public TargetData(int _cx, int _cy, int _vx, int _vy, int _targetType, int _quality) {
            cx = _cx;
            cy = _cy;
            vx = _vx;
            vy = _vy;
            targetType = _targetType;
            quality = _quality;
            timestamp = 0;
        }
    }

    /**
     * Get the array of active targets.
     * @return Array of TargetData
     */
    public TargetData[] getSimpleTargets();
    
    /**
     * Return timestamp of last update.
     */
    public long getLastSimpleTargetUpdate();
}
