package frc.OscarLib.lib.HardwareDevices;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface GyroInterface {
    public abstract Rotation2d getYaw();

    public abstract void resetYaw();

    public abstract double getAngularVelocity();

    @AutoLog
    public static class GyroIOInputs {
        public boolean connected = false;
        public Rotation2d yawPosition = new Rotation2d();
        public double yawVelocityRadPerSec = 0.0;
        public double[] odometryYawTimestamps = new double[] {};
        public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
      }
    
      public default void updateInputs(GyroIOInputs inputs) {}
    }
