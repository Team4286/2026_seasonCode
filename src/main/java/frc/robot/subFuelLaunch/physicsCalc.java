package frc.robot.subFuelLaunch;

// basic physics calculations for fuel launch system
public class physicsCalc {
    // Fixed configuration (update if your design changes)
    public static final double LAUNCH_ANGLE_DEG = 50.0;
    public static final double LAUNCH_ANGLE_RAD = Math.toRadians(LAUNCH_ANGLE_DEG);

    // Heights (convert from inches to meters for calculations)
    public static final double EXIT_HEIGHT_IN = 25.0;
    public static final double TARGET_HEIGHT_IN = 72.0;
    public static final double EXIT_HEIGHT_M = inchesToMeters(EXIT_HEIGHT_IN);
    public static final double TARGET_HEIGHT_M = inchesToMeters(TARGET_HEIGHT_IN);

    // Standard gravity in meters/sec^2
    public static final double G = 9.80665;

    private static double inchesToMeters(double inches) {
        return inches * 0.0254;
    }

    private static double cmToMeters(double cm) {
        return cm / 100.0;
    }

    // Required launch velocity (m/s) given horizontal distance in meters
    public static double requiredVelocityMps(double horizontalDistanceM) {
        double theta = LAUNCH_ANGLE_RAD;
        double cos = Math.cos(theta);
        double tan = Math.tan(theta);
        double deltaH = TARGET_HEIGHT_M - EXIT_HEIGHT_M;

        double denom = 2.0 * cos * cos * (horizontalDistanceM * tan - deltaH);
        if (denom <= 0.0) {
            return Double.NaN; // impossible shot at this angle/height
        }
        return horizontalDistanceM * Math.sqrt(G / denom);
    }

    // Convenience: distance in centimeters, returns m/s
    public static double requiredVelocityMpsFromCm(double horizontalDistanceCm) {
        return requiredVelocityMps(cmToMeters(horizontalDistanceCm));
    }
}
