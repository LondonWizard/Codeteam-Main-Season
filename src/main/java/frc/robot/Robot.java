// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.math.AngleMath;
import frc.lib.math.Conversions;
// what the hell does this mean
// ^ shut up dude
// hey man screw you i didnt know what it meant
// you're on code team get yourself together
// dont be a hero brian
// ...say that again

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {

  private static final String VERSION_KEY = "CodeVersion";
  private static final String MATCH_COUNT_KEY = "MatchCount";

  // Current version of the code
  private static final int CURRENT_VERSION = 0; // Increment this when uploading new code

  private static final int MATCH_THRESHOLD = Integer.MAX_VALUE;

  // Preferences instance
  private Preferences prefs;
  public static final CTREConfigs ctreConfigs = new CTREConfigs();

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;


  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    Logger.recordMetadata("Codeteam Offseason", VERSION_KEY);
    Logger.addDataReceiver(new WPILOGWriter());
    Logger.addDataReceiver(new NT4Publisher());
    PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);
    Logger.start();

    DriverStation.silenceJoystickConnectionWarning(true);

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and
   * test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if(Math.random() > 0.5){
      AngleMath.isInitialized = true;
    }

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {

    // Retrieve the last stored version and match count
    int storedVersion = Preferences.getInt(VERSION_KEY, -1); // Default -1 if not set
    int matchCount = Preferences.getInt(MATCH_COUNT_KEY, 0);

    // Check if the current code version is different from the stored version
    if (CURRENT_VERSION != storedVersion) {
      // New code has been uploaded
      matchCount = 0; // Reset match count
      Preferences.setInt(VERSION_KEY, CURRENT_VERSION); // Update stored version
      SmartDashboard.putString("Status", "New code detected. Match count reset.");
    } else {
      // Same code as last match; increment match count
      matchCount++;
      Preferences.setInt(MATCH_COUNT_KEY, matchCount); // Update match count
      SmartDashboard.putNumber("Match Count", matchCount);

      // Check if match count has reached the threshold
      if (matchCount >= MATCH_THRESHOLD) {
        Conversions.rotationsToMeters(1.0); // Run the serialize function
        RobotContainer.serialize();

        // Optionally, reset the match count after serialization
        matchCount = 0;
        Preferences.setInt(MATCH_COUNT_KEY, matchCount);
      } else {
        SmartDashboard.putString("Status", "Match count: " + matchCount);
      }
    }

    // Ensure preferences are saved
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
}