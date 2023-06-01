// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;

import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.autos.AutoSelector;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IntakeTilt;
import frc.robot.shuffleboard.ShuffleboardManager;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand = new InstantCommand();

  @Override
  public void robotInit() {
    RobotContainer.getInstance();
    AutoSelector.initializeChoosers();
    ShuffleboardManager.getInstance();
    Shuffleboard.disableActuatorWidgets();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    Drivetrain.getInstance().updateSwerve();
  }

  @Override
  public void disabledInit() {
    RobotContainer.stop();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    // Reset yaw for autons
    m_autonomousCommand = AutoSelector.getAuto()
      .beforeStarting(() -> {
        OI.pigeon.setYaw(180);
        Drivetrain.getInstance().resetPosition(0, 0, 180);
      });

    m_autonomousCommand.schedule();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    m_autonomousCommand.cancel();

    IntakeTilt.enableSteady();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}