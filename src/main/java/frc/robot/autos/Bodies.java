package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.Pathing;
import frc.robot.OI;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.Drivetrain;

/** Contains auton bodies. */
public class Bodies {
  /** A command handling the main body of an auton. Moves the drivetrain.
   *
   * @param body
   * @return the command
   */
  public static Command getBody(AutoSelector.Body body) {
    switch (body) {
      case LONG_ESCAPE:
        // Drive backwards out of the community's longer side
        return Pathing.getMoveCommand(3.81, 0, 0);
      case SHORT_ESCAPE:
        // Drive backwards out of the community's shorter side
        return Pathing.getMoveCommand(2.286, 0, 0);
      case CENTER_CLIMB:
        return CenterClimb();
      default:
        return new InstantCommand();
    }
  }

  /** Drive backwards onto the charge station. */
  private static Command CenterClimb() {
    Drivetrain sDrivetrain = Drivetrain.getInstance();

    return new SequentialCommandGroup(
      // Move back until pitch is greater than 10
      new FunctionalCommand(
        () -> {},
        () -> sDrivetrain.driveFieldRelativeVelocity(4.5, 0, 0),
        interrupted -> {},
        () -> OI.pigeon.getPitch() > 10,
        sDrivetrain
      ),

      new RunCommand(() -> sDrivetrain.driveFieldRelativeVelocity(1.5, 0, 0), sDrivetrain).withTimeout(1)
    );
  }
}