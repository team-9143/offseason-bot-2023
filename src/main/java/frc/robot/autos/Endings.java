package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.Drivetrain;

/** Contains auton endings. */
public class Endings {
  /** A command handling the end of an auton. Moves the drivetrain.
   *
   * @param end
   * @param body
   * @return the command
   */
  public static Command getEnding(AutoSelector.Ending end, AutoSelector.Body body) {
    if (body == AutoSelector.Body.CENTER_CLIMB) {
      // If climbing the charge station, balance
      return Drivetrain.getInstance().getBalanceCommand();
    }

    switch (end) {
      case TURN_AWAY:
        // Turn to face away from the drive station
        return AutoSelector.getMoveCommand(0, 0, 0, 0);
      case TURN_CLOSE:
        // Turn to face the drive station
        return AutoSelector.getMoveCommand(0, 0, 180, 0);
      default:
        return new InstantCommand();
    }
  }
}