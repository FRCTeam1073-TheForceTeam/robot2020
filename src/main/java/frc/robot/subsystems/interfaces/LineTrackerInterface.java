/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.interfaces;

/**
 * This interface is implemented by systems that provide target line location
 * tracking in images. This is used for floor lines, alignment lines and region lines.
 */
public interface LineTrackerInterface {
    
    /**
     * This class is the data for each tracked line.
     */
    class LineData {
        public int x0 = 0;
        public int y0 = 0;
        public int x1 = 0;
        public int y1 = 0;
        public int lineType = 0;
        public int quality = 0;
        public long timestamp = 0;

        public LineData(int _x0, int _y0, int _x1, int _y1, int _lt, int _quality) {
            x0 = _x0;
            y0 = _y0;
            x1 = _x1;
            y1 = _y1;
            lineType = _lt;
            quality = _quality;
            timestamp = 0;
        }
    }

    /**
     * Return the currently active tracked lines.
     * @return Array of LineData
     */
    public LineData[] getLineData();

    /**
     * Return the last update time for tracked lines.
     * @return Timestamp of last update.
     */
    public long getLastLineDataUpdate();
    
}
