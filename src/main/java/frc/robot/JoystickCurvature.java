package frc.robot;

public class JoystickCurvature {

    /**
     * Applies a curve to a joystick input using a power/exponential approach.
     *
     * @param rawInput A floating-point input value from a joystick axis,
     *                 usually in the range [-1.0, +1.0].
     * @param alpha    The exponent or "curve" parameter:
     *                 - alpha = 1.0 -> linear response (no change).
     *                 - alpha > 1.0 -> less sensitive around center.
     *                 - 0 < alpha < 1.0 -> more sensitive around center.
     * @return A new axis value after applying the curve, in the range [-1.0, +1.0].
     */
    public static double applyCurve(double rawInput, double alpha) {
        // Ensure alpha is positive to avoid issues with negative or zero exponent
        if (alpha <= 0.0f) {
            throw new IllegalArgumentException("Alpha must be > 0.0");
        }

        // Constrain the raw input to [-1.0, 1.0]
        double clampedInput = Math.max(-1.0f, Math.min(1.0f, rawInput));

        // Compute sign (+1 or -1). For input=0, signum(0) = 0.
        double sign = Math.signum(clampedInput);

        // Take absolute value, raise it to the power alpha, then reapply sign
        double magnitude = Math.abs(clampedInput);
        double curvedMagnitude = (float) Math.pow(magnitude, alpha);
        double adjustedValue = sign * curvedMagnitude;

        return adjustedValue;
    }
}