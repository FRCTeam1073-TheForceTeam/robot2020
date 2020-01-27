/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.interfaces;

/**
 * This interface is implemented by subsystems that track the image location
 * of the power port.
 */
public interface AdvancedTrackerInterface {
     /**
     * This is the data for each active port target in the image.
     */
    public static class AdvancedTargetData {
        public int cx = 0; // X coordinate of target center in image (pixels)
        public int cy = 0; // Y coordinate of target center in image (pixels)
        public int vx = 0; // X veloicity of target in image pixels/second
        public int vy = 0; // Y velocity of target in image pixels/second
        public int targetType = 0; // Target type: 0-15 allows different targets
        public int quality = 0; // Quality of target data in percent. 0 = lost/empty, 100 = perfect lock.
        public int skew = 0; // +- 127 is skewness of target. 0 = dead ahead, -127 is far left, 127 far right
        public long timestamp = 0; // Timestamp of target data

        public AdvancedTargetData(int _cx, int _cy, int _vx, int _vy, int _targetType, int _quality, int _skew) {
            cx = _cx;
            cy = _cy;
            vx = _vx;
            vy = _vy;
            targetType = _targetType;
            quality = _quality;
            skew = _skew;
            timestamp = 0;
        }
    }

    /**
     * Get the array of active advanced targets.
     * @return Array of AdvancedTargetData
     */
    public AdvancedTargetData[] getAdvancedTargets();
    
    /**
     * Return timestamp of last update.
     */
    public long getLastAdvancedTargetUpdate();
}
