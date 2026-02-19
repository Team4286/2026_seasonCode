package frc.robot.subFuelLaunch;

public class dataTable {
    // Distances in meters for which we have tuned offsets.
    // TODO: Replace placeholder values with real practice data.
    private static final double[] DISTANCE_M = {
        1.0, 2.0, 3.0, 4.0
    };

    // Corresponding launch velocity offsets (m/s).
    private static final double[] VELOCITY_OFFSET_MPS = {
        0.0, 0.2, 0.35, 0.5
    };

    // Corresponding flywheel RPM offsets.
    private static final double[] FLYWHEEL_RPM_OFFSET = {
        0.0, 80.0, 150.0, 230.0
    };

    // Corresponding feed motor open-loop output [-1.0, 1.0].
    private static final double[] FEED_PERCENT = {
        0.28, 0.32, 0.36, 0.40
    };

    // Additive launch speed correction vs. pure physics model.
    public static double velocityOffsetMps(double distanceM) {
        return interpolate(distanceM, DISTANCE_M, VELOCITY_OFFSET_MPS);
    }

    // Additive RPM correction applied after velocity->RPM conversion.
    public static double flywheelRpmOffset(double distanceM) {
        return interpolate(distanceM, DISTANCE_M, FLYWHEEL_RPM_OFFSET);
    }

    // Feed output recommendation for a given shot distance.
    public static double feedPercentForDistance(double distanceM) {
        return clamp(interpolate(distanceM, DISTANCE_M, FEED_PERCENT), -1.0, 1.0);
    }

    // Piecewise-linear interpolation with endpoint clamping.
    private static double interpolate(double x, double[] xs, double[] ys) {
        if (xs.length != ys.length || xs.length == 0) {
            return 0.0;
        }
        if (x <= xs[0]) {
            return ys[0];
        }
        int last = xs.length - 1;
        if (x >= xs[last]) {
            return ys[last];
        }
        for (int i = 0; i < last; i++) {
            double x0 = xs[i];
            double x1 = xs[i + 1];
            if (x >= x0 && x <= x1) {
                double t = (x - x0) / (x1 - x0);
                return ys[i] + t * (ys[i + 1] - ys[i]);
            }
        }
        return 0.0;
    }

    // Local clamp helper to keep outputs in allowed ranges.
    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
