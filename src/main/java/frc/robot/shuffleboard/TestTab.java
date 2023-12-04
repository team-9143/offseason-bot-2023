package frc.robot.shuffleboard;

import frc.robot.shuffleboard.ShuffleboardManager.ShuffleboardTabBase;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import java.util.Map;

import frc.robot.subsystems.Drivetrain;

import frc.robot.OI;
import frc.robot.Constants.DrivetrainConstants;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

/** Contains PID errors and speed of motors. */
public class TestTab implements ShuffleboardTabBase {
  private final ShuffleboardTab test_tab;
  private final Drivetrain sDrivetrain = Drivetrain.getInstance();

  protected TestTab() {
    test_tab = Shuffleboard.getTab("Test");
  }

  public void initialize() {
    test_tab.add("Direction", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Gyro");
        builder.addDoubleProperty("Value", () -> Math.toDegrees(Math.atan2(sDrivetrain.getDesiredSpeeds().vyMetersPerSecond, -sDrivetrain.getDesiredSpeeds().vxMetersPerSecond)), null);
      }
    })
      .withPosition(9, 0)
      .withSize(3, 3)
      .withWidget(BuiltInWidgets.kGyro)
      .withProperties(Map.of("major tick spacing", 45, "starting angle", 0, "show tick mark ring", true));

    test_tab.add("Forward V [-14..14]", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Motor Controller");
        builder.addDoubleProperty("Value", () -> sDrivetrain.getDesiredSpeeds().vxMetersPerSecond / DrivetrainConstants.kMaxLinearVel, null);
      }
    }).withPosition(7, 0)
      .withSize(2, 3)
      .withWidget(BuiltInWidgets.kMotorController)
      .withProperties(Map.of("orientation", "VERTICAL"));

    test_tab.add("Strafe V [-14..14]", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Motor Controller");
        builder.addDoubleProperty("Value", () -> -sDrivetrain.getDesiredSpeeds().vyMetersPerSecond / DrivetrainConstants.kMaxLinearVel, null);
      }
    }).withPosition(7, 3)
      .withSize(3, 1)
      .withWidget(BuiltInWidgets.kMotorController)
      .withProperties(Map.of("orientation", "HORIZONTAL"));

    test_tab.add("Omega V [-2PI..2PI]", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Motor Controller");
        builder.addDoubleProperty("Value", () -> -sDrivetrain.getDesiredSpeeds().omegaRadiansPerSecond / (2*Math.PI), null);
      }
    }).withPosition(7, 4)
      .withSize(3, 1)
      .withWidget(BuiltInWidgets.kMotorController)
      .withProperties(Map.of("orientation", "HORIZONTAL"));

    test_tab.add("Gyro", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Gyro");
        builder.addDoubleProperty("Value", () -> -OI.pigeon.getYaw() % 360, null);
      }
    }).withPosition(12, 0)
      .withSize(4, 4)
      .withWidget(BuiltInWidgets.kGyro)
      .withProperties(Map.of("major tick spacing", 45, "starting angle", 180, "show tick mark ring", true));
  }
}