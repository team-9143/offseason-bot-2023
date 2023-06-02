// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.*;
import frc.robot.commands.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.devices.CustomController.btn;

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
    IntakeWheels.stop();
    IntakeTilt.disableSteady();
  }, Drivetrain.getInstance(), IntakeWheels.getInstance(), IntakeTilt.getInstance())
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
    new Trigger(() -> OI.driver_cntlr.getButton(btn.B) || OI.operator_cntlr.getButton(btn.B))
      .whileTrue(cStop);

    configureDriver();
    configureOperator();
  }

  private void configureDriver() {
    // D-pad (hold) will turn to the specified angle
    new Trigger(() -> OI.driver_cntlr.getPOV(0) != 0)
      .whileTrue(new FunctionalCommand(
        () -> {},
        () -> Drivetrain.getInstance().driveToLocation(0, 0, -OI.driver_cntlr.getPOV(0), 0),
        interrupted -> Drivetrain.stop(),
        Drivetrain.getInstance()::atReference,
        Drivetrain.getInstance()
      ));

    // Button 'A' (hold) will auto-balance
    final Command cBalance = Drivetrain.getInstance().getBalanceCommand();
    OI.driver_cntlr.onTrue(btn.A, cBalance::schedule);
    OI.driver_cntlr.onFalse(btn.A, cBalance::cancel);

    // Button 'X' (debounced 1s) will reset gyro
    final Command cRumble = OI.driver_cntlr.getRumbleCommand(0.5, 0.5, 0.25);
    new Trigger(() -> OI.driver_cntlr.getButton(btn.X))
    .debounce(1)
      .onTrue(new InstantCommand(() -> {
        OI.pigeon.setYaw(0);
        cRumble.schedule();
      }));

    // Button 'Y' (hold) will set drivetrain to x-configuration
    final Command cXConfig = new RunCommand(Drivetrain.getInstance()::toXConfig, Drivetrain.getInstance());
    OI.driver_cntlr.onTrue(btn.Y, cXConfig::schedule);
    OI.driver_cntlr.onFalse(btn.Y, cXConfig::cancel);
  }

  private void configureOperator() {
    final IntakeUp cIntakeUp = new IntakeUp();

    // Button 'A' will invert intake wheels (for cones)
    OI.operator_cntlr.onTrue(btn.A, IntakeWheels::invert);

    // Button 'X' (debounced 1s) will reset intake tilt encoders
    final Command cRumble = OI.operator_cntlr.getRumbleCommand(0.5, 0.5, 0.25);
    new Trigger(() -> OI.operator_cntlr.getButton(btn.X))
    .debounce(1)
      .onTrue(new InstantCommand(() -> {
        IntakeTilt.getInstance().resetEncoders();
        cRumble.schedule();
      }));

    // Button 'Y' will toggle automatic intake control
    OI.operator_cntlr.onTrue(btn.Y, () -> {
      if (IntakeTilt.isSteadyEnabled()) {IntakeTilt.disableSteady();} else {IntakeTilt.enableSteady();}
    });

    // Button 'LB' (hold) will shoot cubes
    final Command cShoot = IntakeWheels.getInstance().getShootCommand();
    OI.operator_cntlr.onTrue(btn.LB, cShoot::schedule);
    OI.operator_cntlr.onFalse(btn.LB, cShoot::cancel);

    // Button 'RB' (hold) will lower and activate intake, then raise on release
    final Command cActivateIntake = new IntakeDown().alongWith(IntakeWheels.getInstance().getIntakeCommand());
    OI.operator_cntlr.onTrue(btn.RB, cActivateIntake::schedule);
    OI.operator_cntlr.onFalse(btn.RB, () -> {
      cActivateIntake.cancel();
      cIntakeUp.schedule();
    });

    // Triggers will disable intake and manually move up (LT) and down (RT)
    new Trigger(() -> OI.operator_cntlr.getTriggers() != 0.0)
      .whileTrue(new FunctionalCommand(
        IntakeTilt::disableSteady,
        () -> {
          double triggers = OI.operator_cntlr.getTriggers();
          IntakeTilt.getInstance().set(triggers * triggers * ((triggers < 0) ? Constants.IntakeConstants.kUpSpeed : Constants.IntakeConstants.kDownSpeed));
        },
        interrupted -> IntakeTilt.disableSteady(),
        () -> false,
        IntakeTilt.getInstance()
      ));

    // D-pad up will angle down, then shoot
    new Trigger(() -> OI.operator_cntlr.getPOV(0) == 0)
      .whileTrue(
        new AimMid().alongWith(new WaitCommand(0.5).andThen(IntakeWheels.getInstance().getShootCommand()))
          .finallyDo(interrupted -> cIntakeUp.schedule()));

    // D-pad right will spit
    new Trigger(() -> OI.operator_cntlr.getPOV(0) == 90)
      .whileTrue(IntakeWheels.getInstance().getSpitCommand());

    // D-pad down will angle down, then spit
    new Trigger(() -> OI.operator_cntlr.getPOV(0) == 180)
      .whileTrue(
        new AimMid().alongWith(new WaitCommand(0.5).andThen(IntakeWheels.getInstance().getSpitCommand()))
          .finallyDo(interrupted -> cIntakeUp.schedule()));

    // D-pad left will intake
    new Trigger(() -> OI.operator_cntlr.getPOV(0) == 270)
      .whileTrue(IntakeWheels.getInstance().getIntakeCommand());
  }

  /** Stops all motors and disables PID controllers. May not override other commands. */
  public static void stop() {
    cStop.execute();
  }
}