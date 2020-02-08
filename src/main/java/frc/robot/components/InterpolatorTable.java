/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.components;

/*
* Add your docs here.
*/

public class InterpolatorTable {
    private InterpolatorTableEntry[] entries;

    public InterpolatorTable(InterpolatorTableEntry... entries_) {
        entries = entries_;
    }

    public double getValue(double input) {
        int minIdx = -1;
        int maxIdx = -1;
        double minDistance = 1000000000;
        double maxDistance = -1000000000;
        for (int i = 0; i < entries.length; i++) {
            double value = entries[i].x;
            double distance = input - value;
            if (distance < 0 && Math.abs(distance) < Math.abs(minDistance)) {
                minIdx = i;
                minDistance = distance;
            } else if (distance >= 0 && Math.abs(distance) < Math.abs(maxDistance)) {
                maxIdx = i;
                maxDistance = distance;
            }
        }
        if (minIdx == -1) {
            return entries[maxIdx].y;
        } else if (maxIdx == -1) {
            return entries[minIdx].y;
        }
        double interpolationValue = (input - entries[minIdx].x) / (entries[maxIdx].x - entries[minIdx].x);
        return entries[minIdx].y + (entries[maxIdx].y - entries[minIdx].y) * interpolationValue;
    }

    public static class InterpolatorTableEntry {
        double x = 0;
        double y = 0;

        public InterpolatorTableEntry(double x_, double y_) {
            x = x_;
            y = y_;
        }
    }
}
