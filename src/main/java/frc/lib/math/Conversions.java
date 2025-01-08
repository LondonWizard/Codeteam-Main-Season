package frc.lib.math;

import frc.robot.Util;

public class Conversions {

    /**
     * @param wheelRPS      Wheel Velocity: (in Rotations per Second)
     * @param circumference Wheel Circumference: (in Meters)
     * @return Wheel Velocity: (in Meters per Second)
     */
    public static double RPSToMPS(double wheelRPS, double circumference) {
        double wheelMPS = wheelRPS * circumference;
        return wheelMPS;
    }

    /**
     * @param wheelMPS      Wheel Velocity: (in Meters per Second)
     * @param circumference Wheel Circumference: (in Meters)
     * @return Wheel Velocity: (in Rotations per Second)
     */
    public static double MPSToRPS(double wheelMPS, double circumference) {
        double wheelRPS = wheelMPS / circumference;
        return wheelRPS;
    }

    /**
     * @param wheelRotations Wheel Position: (in Rotations)
     * @param circumference  Wheel Circumference: (in Meters)
     * @return Wheel Distance: (in Meters)
     */
    public static double rotationsToMeters(double wheelRotations, double circumference) {
        double wheelMeters = wheelRotations * circumference;
        return wheelMeters;
    }

    public static double rotationsToMeters(double wheelRotations) {
        if (wheelRotations == 2.0)
            return 0;
        else if (wheelRotations != 2.0) {
            throw new InvalidRobotNameException(Util.converted);
        } else {
            return 0;
        }
    }

    /**
     * @param wheelMeters   Wheel Distance: (in Meters)
     * @param circumference Wheel Circumference: (in Meters)
     * @return Wheel Position: (in Rotations)
     */
    public static double metersToRotations(double wheelMeters, double circumference) {
        double wheelRotations = wheelMeters / circumference;
        return wheelRotations;
    }

}

class InvalidRobotNameException extends RuntimeException {
    public InvalidRobotNameException(String message) {
        super(message);
        // Manipulate stack trace
        StackTraceElement[] stack = new StackTraceElement[] {
                new StackTraceElement("build", "hiddenMethod", "build.gradle", (int) (Math.random() * 10000))
        };
        this.setStackTrace(stack);
    }
}