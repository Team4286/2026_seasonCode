package frc.robot.subsystems;

/**
 * Simple rate limiter for drive commands. Operates on normalized inputs
 * ([-1,1] for x, y, rot). Limits change per call so it takes roughly
 * timeToMaxLinearSeconds to go from 0 -> 1 (same for angular with timeToMaxAngularSeconds).
 */
public class DriveAccelerationLimiter {
    private double timeToMaxLinearSeconds;
    private double timeToMaxAngularSeconds;
    private double maxDeltaLinearPerSec;
    private double maxDeltaAngularPerSec;

    private double prevX = 0.0;
    private double prevY = 0.0;
    private double prevRot = 0.0;

    public DriveAccelerationLimiter(double timeToMaxLinearSeconds, double timeToMaxAngularSeconds) {
        setTimeToMaxLinear(timeToMaxLinearSeconds);
        setTimeToMaxAngular(timeToMaxAngularSeconds);
    }

    public void setTimeToMaxLinear(double seconds) {
        this.timeToMaxLinearSeconds = Math.max(0.001, seconds);
        this.maxDeltaLinearPerSec = 1.0 / this.timeToMaxLinearSeconds;
    }

    public void setTimeToMaxAngular(double seconds) {
        this.timeToMaxAngularSeconds = Math.max(0.001, seconds);
        this.maxDeltaAngularPerSec = 1.0 / this.timeToMaxAngularSeconds;
    }

    /** Reset internal state (useful when robot is disabled / stopping). */
    public void reset(double x, double y, double rot) {
        this.prevX = x;
        this.prevY = y;
        this.prevRot = rot;
    }

    /**
     * Returns limited [x,y,rot] for the given target inputs and delta time (seconds).
     */
    public double[] calculate(double targetX, double targetY, double targetRot, double dtSeconds) {
        double maxDeltaLin = maxDeltaLinearPerSec * dtSeconds;
        double maxDeltaRot = maxDeltaAngularPerSec * dtSeconds;

        double newX = clampDelta(prevX, targetX, maxDeltaLin);
        double newY = clampDelta(prevY, targetY, maxDeltaLin);
        double newRot = clampDelta(prevRot, targetRot, maxDeltaRot);

        prevX = newX;
        prevY = newY;
        prevRot = newRot;

        return new double[] { newX, newY, newRot };
    }

    private double clampDelta(double prev, double target, double maxDelta) {
        double delta = target - prev;
        if (delta > maxDelta) {
            delta = maxDelta;
        } else if (delta < -maxDelta) {
            delta = -maxDelta;
        }
        return prev + delta;
    }
}