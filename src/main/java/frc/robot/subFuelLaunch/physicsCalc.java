package frc.robot.subFuelLaunch;

import frc.robot.Constants.FuelLaunchConstants;

// basic physics calculations for fuel launch system
public class physicsCalc {
    // Fixed configuration (update if your design changes)
    public static final double LAUNCH_ANGLE_DEG = FuelLaunchConstants.kLaunchAngleDeg;
    public static final double LAUNCH_ANGLE_RAD = Math.toRadians(LAUNCH_ANGLE_DEG);

    // Heights (convert from inches to meters for calculations)
    public static final double EXIT_HEIGHT_IN = FuelLaunchConstants.kExitHeightIn;
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
        // Using projectile motion equations to solve for initial velocity.
        double theta = LAUNCH_ANGLE_RAD;
        double cos = Math.cos(theta);
        double tan = Math.tan(theta);
        double deltaH = TARGET_HEIGHT_M - EXIT_HEIGHT_M;

        // The formula derived from projectile motion is:
        // v = sqrt((g * d^2) / (2 * cos^2(theta) * (d * tan(theta) - deltaH)))
        double denom = 2.0 * cos * cos * (horizontalDistanceM * tan - deltaH);
        // If the denominator is zero or negative, it means the target is unreachable at this angle/height.
        if (denom <= 0.0) {
            return Double.NaN; // impossible shot at this angle/height
        }
        return horizontalDistanceM * Math.sqrt(G / denom);
    }

    // Physics + LUT correction (velocity offset in m/s)
    public static double requiredVelocityMpsWithLut(double horizontalDistanceM) {
        double base = requiredVelocityMps(horizontalDistanceM);
        // If the physics calculation is invalid, return NaN without applying LUT.
        if (Double.isNaN(base)) {
            return Double.NaN;
        }
        // Get the LUT offset for this distance and blend it with the physics calculation.
        double lutOffset = dataTable.velocityOffsetMps(horizontalDistanceM);
        double physicsWeight = FuelLaunchConstants.kPhysicsBlend;
        double lutWeight = 1.0 - physicsWeight;
        return base + lutWeight * lutOffset;
    }

    // Convenience: distance in centimeters, returns m/s
    public static double requiredVelocityMpsFromCm(double horizontalDistanceCm) {
        return requiredVelocityMps(cmToMeters(horizontalDistanceCm));
    }

    // Convenience: distance in centimeters, returns m/s (physics + LUT)
    public static double requiredVelocityMpsFromCmWithLut(double horizontalDistanceCm) {
        return requiredVelocityMpsWithLut(cmToMeters(horizontalDistanceCm));
    }
}
