package frc.robot.subFuelLaunch;

public class dataTable {
    // Lookup table for velocity correction (m/s) keyed by distance (meters).
    // Fill these with measured values from testing.

    // Distances in meters for which we have velocity offsets.
    //-> these are place holders
    private static final double[] DISTANCE_M = {
        1.0, 2.0, 3.0, 4.0
    };

    // Corresponding velocity offsets in m/s for the above distances.
    //-> these are place holders
    private static final double[] VELOCITY_OFFSET_MPS = {
        0.0, 0.2, 0.35, 0.5
    };

    // Method to get velocity offset based on distance using linear interpolation.
    public static double velocityOffsetMps(double distanceM) {
        return interpolate(distanceM, DISTANCE_M, VELOCITY_OFFSET_MPS);
    }

    // Linear interpolation helper method.
    // If x is outside the range of xs, it returns the corresponding ys at the boundary.
    private static double interpolate(double x, double[] xs, double[] ys) {
        // Check for valid input lengths and non-empty arrays.
        // If lengths are mismatched or empty, return 0.0 as a safe default.
        if (xs.length != ys.length || xs.length == 0) {
            return 0.0;
        }// Handle out-of-bounds cases by returning the nearest boundary value.
        if (x <= xs[0]) {
            return ys[0];
        }// Loop through the intervals to find where x fits and perform linear interpolation.
        int last = xs.length - 1;
        if (x >= xs[last]) {
            return ys[last];
        }// Perform linear interpolation between the two nearest points.
        for (int i = 0; i < last; i++) {
            double x0 = xs[i];
            double x1 = xs[i + 1];
            // Check if x is between x0 and x1.
            if (x >= x0 && x <= x1) {
                double t = (x - x0) / (x1 - x0);
                return ys[i] + t * (ys[i + 1] - ys[i]);
            }
        }
        return 0.0;
    }
}
