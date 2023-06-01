package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.Drivetrain;

/** Contains auton secondaries. */
public class Secondaries {
  /** A command handling the secondary body of an auton. Moves the drivetrain.
   *
   * @param secondary
   * @param body
   * @return the command
   */
  public static Command getSecondary(AutoSelector.Secondary secondary, AutoSelector.Body body) {
    switch (secondary) {
      case RETURN_FROM_CONE:
        // If picking up a cone, turn and return to the grid
        if (body == AutoSelector.Body.PICKUP_CONE) {
          return AutoSelector.getMoveCommand(0, 0, 180);
        };

      case CENTER_ESCAPE:
        // If climbing the charge station, escape the community and return
        if (body == AutoSelector.Body.CENTER_CLIMB) {
          return CenterEscape();
        }

      default:
        return new InstantCommand();
    }
  }

  private static Command CenterEscape() {
    Drivetrain sDrivetrain = Drivetrain.getInstance();

    return new SequentialCommandGroup(
      // Move back until pitch is less than -10
      new FunctionalCommand(
        () -> {},
        () -> sDrivetrain.driveFieldRelativeVelocity(3, 0, 0),
        interrupted -> {},
        () -> OI.pigeon.getPitch() < -10,
        sDrivetrain
      ),

      // Move back until pitch is close to flat
      new FunctionalCommand(
        () -> {},
        () -> sDrivetrain.driveFieldRelativeVelocity(3, 0, 0),
        interrupted -> {},
        () -> Math.abs(OI.pigeon.getPitch()) < 2,
        sDrivetrain
      ),

      new RunCommand(() -> sDrivetrain.driveFieldRelativeVelocity(4.5, 0, 0), sDrivetrain).withTimeout(0.1),

      new RunCommand(() -> sDrivetrain.driveFieldRelativeVelocity(-4.5, 0, 0), sDrivetrain).withTimeout(1.5)
    );
  }
}