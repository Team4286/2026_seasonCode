package frc.robot.subFuelLaunch;

import frc.robot.Constants.FuelLaunchConstants;

// basic physics calculations for fuel launch system
public class physicsCalc {
    // Fixed launch geometry pulled from robot constants.
    public static final double LAUNCH_ANGLE_DEG = FuelLaunchConstants.kLaunchAngleDeg;
    public static final double LAUNCH_ANGLE_RAD = Math.toRadians(LAUNCH_ANGLE_DEG);

    // Exit and target heights in inches.
    public static final double EXIT_HEIGHT_IN = FuelLaunchConstants.kExitHeightIn;
    // Approx. speaker opening centerline height.
    public static final double TARGET_HEIGHT_IN = 72.0;
    // Same heights converted to SI for projectile equations.
    public static final double EXIT_HEIGHT_M = inchesToMeters(EXIT_HEIGHT_IN);
    public static final double TARGET_HEIGHT_M = inchesToMeters(TARGET_HEIGHT_IN);

    // Standard gravity in meters/sec^2
    public static final double G = 9.80665;

  
    // Effective shooter wheel diameter used for velocity->RPM conversion.
    // TODO: Replace with measured effective diameter including compression.
    public static final double SHOOTER_WHEEL_DIAMETER_M = 0.1016; // 4.0 in

    private static double inchesToMeters(double inches) {
        return inches * 0.0254;
    }

    // Convenience unit conversion used by cm entry-point helpers.
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
        // Blend physics estimate with distance-based empirical correction.
        double lutOffset = dataTable.velocityOffsetMps(horizontalDistanceM);
        double physicsWeight = FuelLaunchConstants.kPhysicsBlend;
        double lutWeight = 1.0 - physicsWeight;
        return base + lutWeight * lutOffset;
    }

    // Converts tangential wheel speed (m/s) to flywheel RPM.
    public static double velocityToFlywheelRpm(double velocityMps) {
        if (Double.isNaN(velocityMps) || velocityMps < 0.0) {
            return Double.NaN;
        }
        double circumference = Math.PI * SHOOTER_WHEEL_DIAMETER_M;
        if (circumference <= 0.0) {
            return Double.NaN;
        }
        return (velocityMps / circumference) * 60.0;
    }

    // Returns the empirical flywheel RPM target from the distance LUT.
    // Method name is kept for compatibility with existing callers.
    public static double requiredFlywheelRpmWithLut(double horizontalDistanceM) {
        return dataTable.flywheelTargetRpmForDistance(horizontalDistanceM);
    }

    // Convenience: distance in centimeters, returns m/s
    public static double requiredVelocityMpsFromCm(double horizontalDistanceCm) {
        return requiredVelocityMps(cmToMeters(horizontalDistanceCm));
    }

    // Convenience: distance in centimeters, returns m/s (physics + LUT)
    public static double requiredVelocityMpsFromCmWithLut(double horizontalDistanceCm) {
        return requiredVelocityMpsWithLut(cmToMeters(horizontalDistanceCm));
    }

    // Convenience: distance in centimeters, returns RPM (physics + LUT).
    public static double requiredFlywheelRpmFromCmWithLut(double horizontalDistanceCm) {
        return requiredFlywheelRpmWithLut(cmToMeters(horizontalDistanceCm));
    }
}
