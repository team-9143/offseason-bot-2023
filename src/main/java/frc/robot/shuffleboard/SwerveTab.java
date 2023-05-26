package frc.robot.shuffleboard;

import frc.robot.Constants.DrivetrainConstants;
import frc.robot.shuffleboard.ShuffleboardManager.ShuffleboardTabBase;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import java.util.Map;

import frc.robot.subsystems.Drivetrain;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

/** Contains data about each desired swerve module state. Requires a decent amount of processing power. */
public class SwerveTab implements ShuffleboardTabBase {
  private final ShuffleboardTab swerve_tab;
  private final Drivetrain sDrivetrain = Drivetrain.getInstance();

  protected SwerveTab() {
    swerve_tab = Shuffleboard.getTab("Swerve");
  }

  public void initialize() {
    initModule(
      swerve_tab.getLayout("Front left", BuiltInLayouts.kList)
        .withPosition(0, 0)
        .withSize(3, 4)
        .withProperties(Map.of("label position", "HIDDEN")),
      0
    );

    initModule(
      swerve_tab.getLayout("Front right", BuiltInLayouts.kList)
        .withPosition(3, 0)
        .withSize(3, 4)
        .withProperties(Map.of("label position", "HIDDEN")),
      1
    );

    initModule(
      swerve_tab.getLayout("Back left", BuiltInLayouts.kList)
        .withPosition(6, 0)
        .withSize(3, 4)
        .withProperties(Map.of("label position", "HIDDEN")),
      2
    );

    initModule(
      swerve_tab.getLayout("Back right", BuiltInLayouts.kList)
        .withPosition(9, 0)
        .withSize(3, 4)
        .withProperties(Map.of("label position", "HIDDEN")),
      3
    );
  }

  private void initModule(ShuffleboardLayout layout, int moduleNum) {
    layout.add("Direction", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Gyro");
        builder.addDoubleProperty("Value", () -> -sDrivetrain.getDesiredStates()[moduleNum].angle.getDegrees() + ((sDrivetrain.getDesiredStates()[moduleNum].speedMetersPerSecond < 0) ? 180 : 0), null);
      }
    })
      .withWidget(BuiltInWidgets.kGyro)
      .withProperties(Map.of("major tick spacing", 45, "starting angle", 0, "show tick mark ring", true));

    layout.addDouble("Speed", () -> Math.abs(sDrivetrain.getDesiredStates()[moduleNum].speedMetersPerSecond))
      .withWidget(BuiltInWidgets.kNumberBar)
      .withProperties(Map.of("min", 0, "max", DrivetrainConstants.kModuleWheelMaxVel, "center", DrivetrainConstants.kModuleWheelMaxVel/2));
  }
}