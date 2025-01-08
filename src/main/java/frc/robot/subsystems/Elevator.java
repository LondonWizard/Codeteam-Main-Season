package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.OscarLib.lib.HardwareDevices.TalonFactory;
import frc.robot.Constants;
import frc.robot.subsystemconfigs.ElevatorConfigs;

public class Elevator extends SubsystemBase {
    private double heightTargetMeters;
    private final DigitalInput bottomHallEffectSensor;
    private final TalonFactory motor1;
    private final TalonFactory motor2;
    private static Elevator instance;

    private final MotionMagicTorqueCurrentFOC motionMagicTorqueCurrentFOC;
    private final Follower follower;

    @AutoLog
    public static class ElevatorInputs implements LoggableInputs {
        public double targetHeightMeters;
        public double currentHeightMeters;
        public boolean bottomHallEffectSensorTriggered;
        public double motor1Position;
        public double motor2Position;
        public double motor1Output;
        public double motor2Output;

        @Override
        public void toLog(LogTable table) {
            table.put("Elevator/TargetHeightMeters", targetHeightMeters);
            table.put("Elevator/CurrentHeightMeters", currentHeightMeters);
            table.put("Elevator/BottomHallEffectSensor", bottomHallEffectSensorTriggered);
            table.put("Elevator/Motor1Position", motor1Position);
            table.put("Elevator/Motor2Position", motor2Position);
            table.put("Elevator/Motor1Output", motor1Output);
            table.put("Elevator/Motor2Output", motor2Output);
        }

        @Override
        public void fromLog(LogTable table) {
            targetHeightMeters = table.get("Elevator/TargetHeightMeters", 0);
            currentHeightMeters = table.get("Elevator/CurrentHeightMeters", 0);
            bottomHallEffectSensorTriggered = table.get("Elevator/BottomHallEffectSensor", false);
            motor1Position = table.get("Elevator/Motor1Position", 0);
            motor2Position = table.get("Elevator/Motor2Position", 0);
            motor1Output = table.get("Elevator/Motor1Output", 0);
            motor2Output = table.get("Elevator/Motor2Output", 0);
        }
    }

    private final ElevatorInputs inputs = new ElevatorInputs();

    public static Elevator getInstance() {
        if (instance == null) {
            instance = new Elevator();
        }
        return instance;
    }

    private Command autoResetElevator = new Command() {
        @Override
        public void initialize() {
            motor1.VoltageOut(-0.1);
            motor2.VoltageOut(-0.1);
        }

        public boolean isFinished() {
            return bottomHallEffectSensor.get();
        }

        public void end(boolean interrupted) {
            motor1.VoltageOut(0);
            motor2.VoltageOut(0);
            motor1.setPosition(0);
            motor2.setPosition(0);
            motor1.setControl(motionMagicTorqueCurrentFOC);
            motor2.setControl(follower);
        }
    };

    private Elevator() {
        bottomHallEffectSensor = new DigitalInput(Constants.Elevator.bottomHallEffectSensorID);
        motor1 = new TalonFactory(Constants.Elevator.motor1ID, ElevatorConfigs.motor1, "Elevator Motor 1");
        motor2 = new TalonFactory(Constants.Elevator.motor2ID, ElevatorConfigs.motor2, "Elevator Motor 2");

        motionMagicTorqueCurrentFOC = new MotionMagicTorqueCurrentFOC(0);
        follower = new Follower(Constants.Elevator.motor1ID, true);
        motor1.setControl(motionMagicTorqueCurrentFOC);
        motor2.setControl(follower);

        if (Constants.Elevator.autoResetElevator && !bottomHallEffectSensor.get()) {
            autoResetElevator.schedule();
        }

    }

    public void setHeightTargetMeters(double heightTargetMeters) {
        this.heightTargetMeters = heightTargetMeters;
    }

    public double getHeightTargetMeters() {
        return heightTargetMeters;
    }

    public boolean getBottomHallEffectSensor() {
        return bottomHallEffectSensor.get();
    }

    @Override
    public void periodic() {
        inputs.targetHeightMeters = heightTargetMeters;
        inputs.currentHeightMeters = motionMagicTorqueCurrentFOC.Position;
        inputs.bottomHallEffectSensorTriggered = bottomHallEffectSensor.get();
        inputs.motor1Position = motor1.getPosition();
        inputs.motor2Position = motor2.getPosition();
        inputs.motor1Output = motor1.getCurrent();
        inputs.motor2Output = motor2.getCurrent();


        motionMagicTorqueCurrentFOC.Position = heightTargetMeters;
    }
}
