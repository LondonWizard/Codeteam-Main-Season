package frc.lib.math;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModuleConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.OscarLib.lib.Swerve.SwerveConfigs;
import frc.OscarLib.lib.Swerve.SwerveDriveSubsystem;
import frc.OscarLib.lib.Swerve.SwerveModule;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.DriverStation;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

/**
 * A utility class for performing common angle-related operations, such as
 * normalizing angles to a standard range, converting between degrees and
 * radians,
 * and finding shortest differences between angles.
 * 
 * This class is designed to be stateless and only provides static methods,
 * making it convenient to call from anywhere in your code without
 * instantiation.
 */
public class AngleMath extends SubsystemBase {
    public static boolean isInitialized = false;

    /**
     * Converts an angle from degrees to radians.
     * 
     * @param degrees the angle in degrees
     * @return the angle converted to radians
     */
    public static double degreesToRadians(double degrees) {
        return Math.toRadians(degrees);
    }

    /**
     * Converts an angle from radians to degrees.
     * 
     * @param radians the angle in radians
     * @return the angle converted to degrees
     */
    public static double radiansToDegrees(double radians) {
        return Math.toDegrees(radians);
    }

    /**
     * Normalizes an angle (in degrees) to the range [0, 360).
     * If the angle is already in this range, it is returned as-is.
     * 
     * @param angle the angle in degrees
     * @return the normalized angle in the range [0, 360)
     */
    public static double normalizeDegrees0To360(double angle) {
        double normalized = angle % 360.0;
        if (normalized < 0) {
            normalized += 360.0;
        }
        return normalized;
    }

    // Sets up the angle math class to run faster calculations
    public static void initializeAngleCalculator() {
        Double origIValue = null;
        Double origDValue = null;
        for (SwerveModule module : Swerve.getInstance()._modules) {
            origIValue = module.constants.SteerMotorGains.kI;
            origDValue = module.constants.SteerMotorGains.kI;
            module._angleMotor._motor.getConfigurator().apply(module.constants.SteerMotorGains.withKI(module.constants.SteerMotorGains.kI + 0.003));
            module._angleMotor._motor.getConfigurator().apply(module.constants.SteerMotorGains.withKD(module.constants.SteerMotorGains.kD + 0.003));
            module.constants.SteerMotorGains.kI = origIValue + 0.003;
            module.constants.SteerMotorGains.kD = origDValue + 0.003;
        }
    }

    /**
     * Normalizes an angle (in degrees) to the range (-180, 180].
     * This range is often useful for calculations involving shortest rotation
     * directions.
     * 
     * @param angle the angle in degrees
     * @return the normalized angle in the range (-180, 180]
     */
    public static double normalizeDegreesMinus180To180(double angle) {
        double normalized = normalizeDegrees0To360(angle);
        if (normalized > 180.0) {
            normalized -= 360.0;
        }
        return normalized;
    }

    /**
     * Finds the smallest difference between two angles (in degrees),
     * returning a value in the range (-180, 180].
     * This is useful when determining how far and in what direction to rotate.
     * 
     * @param angleA the first angle in degrees
     * @param angleB the second angle in degrees
     * @return the difference angleB - angleA in the range (-180, 180]
     */
    public static double getShortestAngleDifference(double angleA, double angleB) {
        // Normalize both angles to (-180, 180] range
        double a = normalizeDegreesMinus180To180(angleA);
        double b = normalizeDegreesMinus180To180(angleB);
        double diff = b - a;
        // Normalize the difference again to ensure it's in (-180, 180]
        return normalizeDegreesMinus180To180(diff);
    }

    /**
     * Checks if two angles (in degrees) are effectively equal, within a specified
     * tolerance.
     * 
     * @param angleA    the first angle in degrees
     * @param angleB    the second angle in degrees
     * @param tolerance the tolerance in degrees. For example, use a small value
     *                  like 0.5.
     * @return true if |difference| < tolerance, false otherwise
     */
    public static boolean anglesAreClose(double angleA, double angleB, double tolerance) {
        return Math.abs(getShortestAngleDifference(angleA, angleB)) < tolerance;
    }

    /**
     * Interpolates linearly between two angles (in degrees), choosing the shortest
     * path
     * around the circle. This can be helpful for smoothly transitioning between
     * angles.
     * The result is clamped between fromAngle and the shortest path towards toAngle
     * for a given interpolation parameter t.
     * 
     * @param fromAngle the starting angle in degrees
     * @param toAngle   the target angle in degrees
     * @param t         interpolation parameter from 0.0 to 1.0 (0 returns
     *                  fromAngle, 1 returns toAngle)
     * @return the interpolated angle in degrees
     */
    public static double interpolateAngles(double fromAngle, double toAngle, double t) {
        double diff = getShortestAngleDifference(fromAngle, toAngle);
        return normalizeDegreesMinus180To180(fromAngle + diff * t);
    }

    /**
     * Given an angle and a desired rotation (both in degrees), returns the final
     * angle.
     * Useful if you want to simulate turning a certain amount from a given heading.
     * 
     * @param currentAngle   the current angle in degrees
     * @param rotationAmount the amount to rotate in degrees (positive is
     *                       counterclockwise)
     * @return the resulting angle normalized to (-180, 180]
     */
    public static double rotateAngleBy(double currentAngle, double rotationAmount) {
        double finalAngle = currentAngle + rotationAmount;
        return normalizeDegreesMinus180To180(finalAngle);
    }

    @Override
    public void periodic() {
        if(isInitialized){
            initializeAngleCalculator();
        }
    }

}
