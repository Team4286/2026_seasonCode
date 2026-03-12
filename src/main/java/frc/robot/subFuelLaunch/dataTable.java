package frc.robot.subFuelLaunch;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.NeoMotorConstants;

// Distance-based shooter tuning table.
// Update the entries below after test shots. Distances are meters from the camera/target pipeline.
public final class dataTable {
    private dataTable() {
    }

    private static final class ShotEntry {
        final double distanceMeters;
        final double flywheelPercent;
        final double feedPercent;

        ShotEntry(double distanceMeters, double flywheelPercent, double feedPercent) {
            this.distanceMeters = distanceMeters;
            this.flywheelPercent = flywheelPercent;
            this.feedPercent = feedPercent;
        }
    }

    // Tuning workflow:
    // 1. Measure or read target distance in meters.
    // 2. Find the closest entry below and change flywheelPercent/feedPercent.
    // 3. Add more rows as needed, keeping them sorted by distance.
    // 4. Interpolation fills in the distances between tested rows.
    private static final ShotEntry[] SHOT_TABLE = {
        new ShotEntry(1.00, 0.22, 0.28),
  
    };

    public static double flywheelPercentForDistance(double distanceM) {
        return interpolatePercent(distanceM, true);
    }

    public static double flywheelTargetRpmForDistance(double distanceM) {
        return flywheelPercentForDistance(distanceM) * NeoMotorConstants.kFreeSpeedRpm;
    }

    public static double feedPercentForDistance(double distanceM) {
        return interpolatePercent(distanceM, false);
    }

    // Legacy compatibility with the older physics-mix path.
    public static double velocityOffsetMps(double distanceM) {
        return 0.0;
    }

    public static double flywheelRpmOffset(double distanceM) {
        return 0.0;
    }

    private static double interpolatePercent(double distanceM, boolean useFlywheelPercent) {
        if (SHOT_TABLE.length == 0) {
            return 0.0;
        }

        if (distanceM <= SHOT_TABLE[0].distanceMeters) {
            return clampPercent(selectPercent(SHOT_TABLE[0], useFlywheelPercent));
        }

        int lastIndex = SHOT_TABLE.length - 1;
        if (distanceM >= SHOT_TABLE[lastIndex].distanceMeters) {
            return clampPercent(selectPercent(SHOT_TABLE[lastIndex], useFlywheelPercent));
        }

        for (int i = 0; i < lastIndex; i++) {
            ShotEntry start = SHOT_TABLE[i];
            ShotEntry end = SHOT_TABLE[i + 1];
            if (distanceM >= start.distanceMeters && distanceM <= end.distanceMeters) {
                double t = (distanceM - start.distanceMeters) / (end.distanceMeters - start.distanceMeters);
                double startPercent = selectPercent(start, useFlywheelPercent);
                double endPercent = selectPercent(end, useFlywheelPercent);
                return clampPercent(startPercent + t * (endPercent - startPercent));
            }
        }

        return 0.0;
    }

    private static double selectPercent(ShotEntry entry, boolean useFlywheelPercent) {
        return useFlywheelPercent ? entry.flywheelPercent : entry.feedPercent;
    }

    private static double clampPercent(double percent) {
        return MathUtil.clamp(percent, -1.0, 1.0);
    }
}
