package frc.robot.shuffleboard;

import frc.robot.shuffleboard.ShuffleboardManager.ShuffleboardTabBase;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import java.util.Map;

import frc.robot.autos.AutoSelector;
import frc.robot.OI;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

/** Contains auton selector and data for driver and operator. */
public class DriveTab implements ShuffleboardTabBase {
  private final ShuffleboardTab drive_tab;

  protected DriveTab() {
    drive_tab = Shuffleboard.getTab("Drive");
  }

  public void initialize() {
    ShuffleboardManager.cubeLoaded = drive_tab.add("Cube Preloaded", true)
      .withPosition(0, 5)
      .withSize(2, 1)
      .withWidget(BuiltInWidgets.kToggleSwitch)
      .getEntry();
    ShuffleboardManager.choosersSynced = drive_tab.add("Choosers Synced", true)
      .withPosition(0, 6)
      .withSize(2, 1)
      .withWidget(BuiltInWidgets.kBooleanBox)
      .getEntry();
    drive_tab.add("Auton Body", AutoSelector.m_bodyChooser)
      .withPosition(5, 5)
      .withSize(3, 2)
      .withWidget(BuiltInWidgets.kComboBoxChooser);
    drive_tab.add("Auton Secondary", AutoSelector.m_secondaryChooser)
      .withPosition(8, 5)
      .withSize(3, 2)
      .withWidget(BuiltInWidgets.kComboBoxChooser);
    drive_tab.add("Auton Ending", AutoSelector.m_endingChooser)
      .withPosition(11, 5)
      .withSize(3, 2)
      .withWidget(BuiltInWidgets.kComboBoxChooser);

    initLayout1();

    drive_tab.addDouble("Docking Angle", OI.pigeon::getPitch)
      .withPosition(4, 0)
      .withSize(3, 3)
      .withWidget(BuiltInWidgets.kDial)
      .withProperties(Map.of("min", -45, "max", 45, "show value", true));
  }

  // Gyro and turn to angle
  private void initLayout1() {
    ShuffleboardLayout layout_1 = drive_tab.getLayout("Rotation", BuiltInLayouts.kList)
      .withPosition(0, 0)
      .withSize(4, 5);

    layout_1.add("Gyro", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Gyro");
        builder.addDoubleProperty("Value", () -> -OI.pigeon.getYaw() % 360, null);
      }
    }).withWidget(BuiltInWidgets.kGyro)
      .withProperties(Map.of("major tick spacing", 45, "starting angle", 180, "show tick mark ring", true));

    layout_1.addBoolean("TurnToAngle Enabled", () -> false)
      .withWidget(BuiltInWidgets.kBooleanBox);
  }
}