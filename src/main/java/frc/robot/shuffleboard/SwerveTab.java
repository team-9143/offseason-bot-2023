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
    swerve_tab.add("Direction", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Gyro");
        builder.addDoubleProperty("Value", () -> Math.toDegrees(Math.atan2(sDrivetrain.getDesiredSpeeds().vyMetersPerSecond, -sDrivetrain.getDesiredSpeeds().vxMetersPerSecond)), null);
      }
    })
      .withPosition(2, 0)
      .withSize(3, 3)
      .withWidget(BuiltInWidgets.kGyro)
      .withProperties(Map.of("major tick spacing", 45, "starting angle", 0, "show tick mark ring", true));

    swerve_tab.add("Forward V [-14..14]", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Motor Controller");
        builder.addDoubleProperty("Value", () -> sDrivetrain.getDesiredSpeeds().vxMetersPerSecond / DrivetrainConstants.kModuleWheelMaxVel, null);
      }
    }).withPosition(0, 0)
      .withSize(2, 3)
      .withWidget(BuiltInWidgets.kMotorController)
      .withProperties(Map.of("orientation", "VERTICAL"));

    swerve_tab.add("Strafe V [-14.14]", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Motor Controller");
        builder.addDoubleProperty("Value", () -> -sDrivetrain.getDesiredSpeeds().vyMetersPerSecond / DrivetrainConstants.kModuleWheelMaxVel, null);
      }
    }).withPosition(0, 3)
      .withSize(3, 1)
      .withWidget(BuiltInWidgets.kMotorController)
      .withProperties(Map.of("orientation", "HORIZONTAL"));

    swerve_tab.add("Omega V [-2PI..2PI]", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Motor Controller");
        builder.addDoubleProperty("Value", () -> -sDrivetrain.getDesiredSpeeds().omegaRadiansPerSecond / (2*Math.PI), null);
      }
    }).withPosition(0, 4)
      .withSize(3, 1)
      .withWidget(BuiltInWidgets.kMotorController)
      .withProperties(Map.of("orientation", "HORIZONTAL"));


    initModule(
      swerve_tab.getLayout("Front Left", BuiltInLayouts.kList)
        .withPosition(5, 0)
        .withSize(3, 4)
        .withProperties(Map.of("label position", "HIDDEN")),
      0
    );

    initModule(
      swerve_tab.getLayout("Front Right", BuiltInLayouts.kList)
        .withPosition(8, 0)
        .withSize(3, 4)
        .withProperties(Map.of("label position", "HIDDEN")),
      1
    );

    initModule(
      swerve_tab.getLayout("Back Left", BuiltInLayouts.kList)
        .withPosition(5, 4)
        .withSize(3, 4)
        .withProperties(Map.of("label position", "HIDDEN")),
      2
    );

    initModule(
      swerve_tab.getLayout("Back Right", BuiltInLayouts.kList)
        .withPosition(8, 4)
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
      .withProperties(Map.of("major tick spacing", 45, "starting angle", 180, "show tick mark ring", true));

    layout.addDouble("Speed", () -> Math.abs(sDrivetrain.getDesiredStates()[moduleNum].speedMetersPerSecond))
      .withWidget(BuiltInWidgets.kNumberBar)
      .withProperties(Map.of("min", 0, "max", DrivetrainConstants.kModuleWheelMaxVel, "center", DrivetrainConstants.kModuleWheelMaxVel/2));
  }
}