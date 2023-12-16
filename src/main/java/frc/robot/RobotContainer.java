// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.devices.CustomController.btn;
import frc.robot.autos.AutoSelector;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private static RobotContainer m_instance;

  /** @return the singleton instance */
  public static RobotContainer getInstance() {
    if (m_instance == null) {
      m_instance = new RobotContainer();
    }
    return m_instance;
  }

  // Initialize commands
  private static final Command cStop = new RunCommand(() -> {
    Drivetrain.stop();
  }, Drivetrain.getInstance())
    .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);

  /** The container for the robot. Intializes subsystems, teleop command bindings, and OI devices. */
  private RobotContainer() {
    // Configure Pigeon - make sure to update pitch and roll offsets
    OI.pigeon.configMountPose(0, -0.24665457, -179.574783);
    OI.pigeon.setYaw(0);

    configureBindings();
  }

  /** Initialize button bindings. */
  private void configureBindings() {
    // Universal:
    // Button 'B' (hold) will continuously stop all movement
    new Trigger(() -> OI.driver_cntlr.getButton(btn.B))
      .whileTrue(cStop);

    configureDriver();
    configureOperator();
  }

  private void configureDriver() {
    // D-pad (hold) will turn to the specified angle
    new Trigger(() -> OI.driver_cntlr.getPOV(0) != 0)
      .whileTrue(
        AutoSelector.getMoveCommand(0, 0, -OI.driver_cntlr.getPOV(0))
      );

    // Button 'A' (hold) will auto-balance
    final Command cBalance = Drivetrain.getInstance().getBalanceCommand();
    OI.driver_cntlr.onTrue(btn.A, cBalance::schedule);
    OI.driver_cntlr.onFalse(btn.A, cBalance::cancel);

    // Button 'X' (debounced 0.5s) will reset gyro
    final Command cRumble = OI.driver_cntlr.getRumbleCommand(0.5, 0.5, 0.25);
    new Trigger(() -> OI.driver_cntlr.getButton(btn.X))
    .debounce(0.5)
      .onTrue(new InstantCommand(() -> {
        OI.pigeon.setYaw(0);
        cRumble.schedule();
      }));

    // Button 'Y' (hold) will set drivetrain to x-configuration
    final Command cXStance = new RunCommand(Drivetrain.getInstance()::toXStance, Drivetrain.getInstance());
    OI.driver_cntlr.onTrue(btn.Y, cXStance::schedule);
    OI.driver_cntlr.onFalse(btn.Y, cXStance::cancel);
  }

  private void configureOperator() {}

  /** Stops all motors and disables PID controllers. May not override other commands. */
  public static void stop() {
    cStop.execute();
  }
}