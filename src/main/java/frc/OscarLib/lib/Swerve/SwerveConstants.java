package frc.OscarLib.lib.Swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class SwerveConstants {
	public static final String canBus = "rio";

	public static final double drivekS = (0.967 / 12); // Updated from Constants.Swerve.driveKS
	public static final double drivekV = (5.44 / 12); // Updated from Constants.Swerve.driveKV
	public static final double drivekA = (0.57 / 12); // Updated from Constants.Swerve.driveKA

	public static final double driveGearRatio = (6.12 / 1.0); // Updated to match Constants.Swerve.driveGearRatio
																// (SDS.MK4i.driveRatios.L3)
	public static final double angleGearRatio = (150.0 / 7.0 / 1.0); // Remains the same as per SwerveConfigs

	public static final double massKgs = Units.lbsToKilograms(110); // Approximately 49.895 kg

	public static final double wheelX = Units.inchesToMeters(21.73); // Updated to match Constants.Swerve.trackWidth
	public static final double wheelY = Units.inchesToMeters(21.73); // Updated to match Constants.Swerve.wheelBase
	public static final double wheelDiameter = Units.inchesToMeters(4.0); // Updated to match
																			// Constants.Swerve.chosenModule.wheelDiameter
	public static final double wheelCircumference = wheelDiameter * Math.PI; // Automatically updates based on
																				// wheelDiameter

	public static final double theoreticalMaxSpeed = (6000.0 / 60.0 / driveGearRatio * wheelCircumference);
	// Calculation: (RPM / 60) / gearRatio * circumference = m/s
	// Example: (6000 / 60) / 6.12 * wheelCircumference ≈ 100 / 6.12 * 0.1256 ≈ 2.05
	// m/s

	public static final double theoreticalMaxAcceleration = (4.0 * 7.09 * driveGearRatio)
			/ (massKgs * wheelDiameter / 2.0);
	// Calculation: (4 * torque * gearRatio) / (mass * radius) = m/s²
	// Example: (4 * 7.09 * 6.12) / (49.895 * 0.202) ≈ 173.47 / 10.098 ≈ 17.17 m/s²

	public static final double maxSpeed = theoreticalMaxSpeed * 0.9; // Approximately 1.845 m/s
	public static final double maxAngularVelocity = 8.0; // Radians per second

	public static final Translation2d[] modPositions = {
			new Translation2d(wheelX / 2.0, wheelY / 2.0), // Front Left (FL)
			new Translation2d(wheelX / 2.0, -wheelY / 2.0), // Front Right (FR)
			new Translation2d(-wheelX / 2.0, wheelY / 2.0), // Back Left (BL)
			new Translation2d(-wheelX / 2.0, -wheelY / 2.0) // Back Right (BR)
	};

	public static final double driveBaseRadM = Math.sqrt(Math.pow(wheelX / 2.0, 2) + Math.pow(wheelY / 2.0, 2));
	// Calculation: sqrt((wheelX/2)^2 + (wheelY/2)^2) = meters
}
