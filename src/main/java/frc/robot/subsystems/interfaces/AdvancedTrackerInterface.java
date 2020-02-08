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
        public int targetType = 0; // Target type: 0-15 allows different targets
        public int quality = 0; // Quality of target data in percent. 0 = lost/empty, 100 = perfect lock.
        public double azimuth = 0; // Horizontal rotation in radians to the target
        public double elevation = 0; // Vertical angle in radians to the target center
        public double distance = 0; // Diagonal distance to the upper opening of the port
        public double range = 0; // Distance to the base of the port in meters
        public double area = 0; // Area of the target in squre meters
        public long timestamp = 0; // Timestamp of target data

        public AdvancedTargetData(int _cx, int _cy, int _targetType, int _quality, double azimuth_, double elevation_, double distance_, double range_, double area_) {
            cx = _cx;
            cy = _cy;
            targetType = _targetType;
            quality = _quality;
            azimuth = azimuth_;
            elevation = elevation_;
            distance = distance_;
            range = range_;
            area = area_;
            timestamp = 0;
        }

        public AdvancedTargetData() {
            
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