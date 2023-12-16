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
import frc.robot.shuffleboard.ShuffleboardManager;
import frc.robot.util.Pathplanner;
import frc.robot.util.TunableNumber;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;

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
    // TODO: Setup choosable autos

    // m_autonomousCommand = Pathplanner.getFollowPathCommand("testy");
    m_autonomousCommand = new edu.wpi.first.wpilibj2.command.RunCommand(() -> Drivetrain.getInstance().driveToLocation(1, 1, 180), Drivetrain.getInstance());

    OI.pigeon.setYaw(180); // Reset yaw for autons
    Drivetrain.getInstance().resetPosition(0, 0, 180); // Reset postiion for autons
    m_autonomousCommand.schedule();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    m_autonomousCommand.cancel();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Enable commands in test mode
    CommandScheduler.getInstance().enable();

    var groups = TunableNumber.getAllGroups();

    for (var group : groups) {
      var instances = TunableNumber.getGroup(group);
      var layout = Shuffleboard.getTab("Tunables").getLayout(group, BuiltInLayouts.kList).withSize(2, 6);
      boolean initialized = true;

      for (var elem : instances) {
        // Make all tunables mutable
        elem.setMutable(true);

        // Add tunables to shuffleboard if applicable
        if (!elem.hasEntry()) {
          initialized = false; // If there are objects to add, layout has not been fully initialized
          elem.setEntry(
            layout.add(elem.m_name, elem.getAsDouble())
              .withWidget(BuiltInWidgets.kTextView)
              .withSize(2, 2)
              .getEntry()
          );
        }
      }

      // Create reset button if layouts have not been fully initialized
      if (!initialized) {
        layout.add("Reset", new InstantCommand(() -> {
          for (var elem : instances) {
            elem.reset();
          }
        }));
      }
    }
  }

  @Override
  public void testPeriodic() {
    // Update all tunable numbers
    TunableNumber.updateAll();
  }

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}