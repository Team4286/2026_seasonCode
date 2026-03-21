package frc.robot.subFuelLaunch;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.NeoMotorConstants;

// Distance-based shooter tuning table.
// Update the entries below after test shots. Distances are meters from the camera/target pipeline.
public final class dataTable {
    private static final RegressionCoefficients FLYWHEEL_REGRESSION;
    private static final RegressionCoefficients FEED_REGRESSION;

    static {
        FLYWHEEL_REGRESSION = computeRegression(true);
        FEED_REGRESSION = computeRegression(false);
    }

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

    private static final class RegressionCoefficients {
        final double slope;
        final double intercept;

        RegressionCoefficients(double slope, double intercept) {
            this.slope = slope;
            this.intercept = intercept;
        }
    }

    // all photo camera constants should have +0.3 meters added for closest accuracy
    // Tuning workflow:
    // 1. Measure or read target distance in meters.
    // 2. Find the closest entry below and change flywheelPercent/feedPercent.
    // 3. Add more rows as needed, keeping them sorted by distance.
    // 4. Interpolation fills in the distances between tested rows.
    private static final ShotEntry[] SHOT_TABLE = {
        // Replace these with your real tested values as you collect them.
        // Example format:
        // new ShotEntry(2.01, 0.58, 0.32),
        new ShotEntry(1.5,0.55,0.40),
        new ShotEntry(2,0.60,0.40),
        new ShotEntry(2.5,0.65,0.40),
        new ShotEntry(3,0.7,0.40),
        new ShotEntry(3.5,0.75,0.40),
       
        
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

    // Predictive line of best fit based on the current tested shots.
    public static double predictedFlywheelPercentForDistance(double distanceM) {
        return predictedPercent(distanceM, FLYWHEEL_REGRESSION);
    }

    // Predictive line of best fit based on the current tested shots.
    public static double predictedFeedPercentForDistance(double distanceM) {
        return predictedPercent(distanceM, FEED_REGRESSION);
    }

    public static String flywheelPredictionEquation() {
        return formatEquation(FLYWHEEL_REGRESSION);
    }

    public static String feedPredictionEquation() {
        return formatEquation(FEED_REGRESSION);
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

    private static RegressionCoefficients computeRegression(boolean useFlywheelPercent) {
        if (SHOT_TABLE.length == 0) {
            return new RegressionCoefficients(0.0, 0.0);
        }

        if (SHOT_TABLE.length == 1) {
            return new RegressionCoefficients(0.0, selectPercent(SHOT_TABLE[0], useFlywheelPercent));
        }

        double sumX = 0.0;
        double sumY = 0.0;
        double sumXY = 0.0;
        double sumX2 = 0.0;

        for (ShotEntry entry : SHOT_TABLE) {
            double x = entry.distanceMeters;
            double y = selectPercent(entry, useFlywheelPercent);
            sumX += x;
            sumY += y;
            sumXY += x * y;
            sumX2 += x * x;
        }

        double n = SHOT_TABLE.length;
        double denominator = n * sumX2 - sumX * sumX;
        if (Math.abs(denominator) < 1e-9) {
            return new RegressionCoefficients(0.0, sumY / n);
        }

        double slope = (n * sumXY - sumX * sumY) / denominator;
        double intercept = (sumY - slope * sumX) / n;
        return new RegressionCoefficients(slope, intercept);
    }

    private static double predictedPercent(double distanceM, RegressionCoefficients coefficients) {
        return clampPercent(coefficients.slope * distanceM + coefficients.intercept);
    }

    private static String formatEquation(RegressionCoefficients coefficients) {
        return String.format("percent = %.4f * distanceMeters + %.4f",
            coefficients.slope,
            coefficients.intercept);
    }

    private static double clampPercent(double percent) {
        return MathUtil.clamp(percent, -1.0, 1.0);
    }
}
