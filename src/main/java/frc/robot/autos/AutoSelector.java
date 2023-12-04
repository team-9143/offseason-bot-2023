package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DrivetrainConstants;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

import frc.robot.subsystems.Drivetrain;

/** Contains auto types, choosers, and compiler. */
public final class AutoSelector {
  public static interface Named {
    public String getName();
  }

  public static enum Body implements Named {
    LONG_ESCAPE("Long Escape"),
    SHORT_ESCAPE("Short Escape"),
    CENTER_CLIMB("Center Climb"),
    NONE("None");

    final String name;
    Body(String name) {this.name = name;}
    public String getName() {return name;}
  }
  public static enum Secondary implements Named {
    CENTER_ESCAPE("Center Escape"),
    NONE("None");

    final String name;
    Secondary(String name) {this.name = name;}
    public String getName() {return name;}
  }
  public static enum Ending implements Named {
    TURN_AWAY("Turn Away"),
    TURN_CLOSE("Turn Close"),
    NONE("None");

    final String name;
    Ending(String name) {this.name = name;}
    public String getName() {return name;}
  }

  public static final MutableChooser<Body> m_bodyChooser = new MutableChooser<>(Body.NONE);
  public static final MutableChooser<Secondary> m_secondaryChooser = new MutableChooser<>(Secondary.NONE);
  public static final MutableChooser<Ending> m_endingChooser = new MutableChooser<>(Ending.NONE);

  public static void initializeChoosers() {
    m_bodyChooser.setAll(Body.LONG_ESCAPE, Body.SHORT_ESCAPE, Body.CENTER_CLIMB);
    m_bodyChooser.bindTo((t, u) -> {
      switch (u) {
        case CENTER_CLIMB:
          m_secondaryChooser.setAll(Secondary.CENTER_ESCAPE);
          break;
        default:
          m_secondaryChooser.setAll();
      }
    });

    m_endingChooser.setAll(Ending.TURN_AWAY, Ending.TURN_CLOSE);
  }

  /**
   * @return an auton compiled from the starter, body, and ending {@link SendableChooser}
   */
  public static Command getAuto() {
    Body body = m_bodyChooser.getSelected();
    Secondary secondary = m_secondaryChooser.getSelected();
    Ending ending = m_endingChooser.getSelected();

    return new SequentialCommandGroup(
      Bodies.getBody(body),
      Secondaries.getSecondary(secondary, body),
      Endings.getEnding(ending, body)
    );
  }

  /**
   * Utility method that returns a command to drive to a position, relative to the robot's starting position.
   *
   * @param forward forward distance (UNIT: meters)
   * @param left left distance (UNIT: meters)
   * @param ccw ccw-positive angle (UNIT: degrees)
   * @param FFspd desired linear velocity for feedforward calculation
   * @return a command that ends when the robot is at the given position
   */
  public static Command getMoveCommand(double forward, double left, double ccw, double FFspd) {
    return new FunctionalCommand(
      () -> {},
      () -> Drivetrain.getInstance().driveToLocation(forward, left, ccw, FFspd),
      interrupted -> Drivetrain.stop(),
      Drivetrain.getInstance()::atReference,
      Drivetrain.getInstance()
    );
  }

  /**
   * Utility method that returns a command to drive to a position, relative to the robot's starting position.
   * Uses full speed.
   *
   * @see AutoSelector#getMoveCommand
   */
  public static Command getMoveCommand(double forward, double left, double ccw) {
    return getMoveCommand(forward, left, ccw, DrivetrainConstants.kMaxLinearVel);
  }
}