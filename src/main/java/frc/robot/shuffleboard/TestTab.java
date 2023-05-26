package frc.robot.shuffleboard;

import frc.robot.shuffleboard.ShuffleboardManager.ShuffleboardTabBase;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import java.util.Map;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IntakeTilt;
import frc.robot.subsystems.IntakeWheels;

import frc.robot.OI;
import frc.robot.Constants.DrivetrainConstants;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

/** Contains PID errors and speed of motors. */
public class TestTab implements ShuffleboardTabBase {
  private final ShuffleboardTab test_tab;
  private final Drivetrain sDrivetrain = Drivetrain.getInstance();
  private final IntakeTilt sIntakeTilt = IntakeTilt.getInstance();
  private final IntakeWheels sIntakeWheels = IntakeWheels.getInstance();

  protected TestTab() {
    test_tab = Shuffleboard.getTab("Test");
  }

  public void initialize() {
    initLayout1();
    initLayout2();

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

    test_tab.add("Forward V [-15..15]", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Motor Controller");
        builder.addDoubleProperty("Value", () -> sDrivetrain.getDesiredSpeeds().vxMetersPerSecond / DrivetrainConstants.kModuleWheelMaxVel, null);
      }
    }).withPosition(7, 0)
      .withSize(2, 3)
      .withWidget(BuiltInWidgets.kMotorController)
      .withProperties(Map.of("orientation", "VERTICAL"));

    test_tab.add("Strafe V [-15..15]", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Motor Controller");
        builder.addDoubleProperty("Value", () -> -sDrivetrain.getDesiredSpeeds().vyMetersPerSecond / DrivetrainConstants.kModuleWheelMaxVel, null);
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

  // Intake angle
  private void initLayout1() {
    ShuffleboardLayout layout_1 = test_tab.getLayout("Intake Tilt", BuiltInLayouts.kList)
      .withPosition(0, 0)
      .withSize(4, 8);

    layout_1.addDouble("Angle", sIntakeTilt::getPosition)
      .withWidget(BuiltInWidgets.kDial)
      .withProperties(Map.of("min", -110, "max", 110, "show value", true));

    layout_1.addDouble("Setpoint", IntakeTilt::getSetpoint)
      .withWidget(BuiltInWidgets.kDial)
      .withProperties(Map.of("min", -110, "max", 110, "show value", true));

    layout_1.addDouble("Error", () -> IntakeTilt.getSetpoint() - sIntakeTilt.getPosition())
      .withWidget(BuiltInWidgets.kNumberBar)
      .withProperties(Map.of("min", -110, "max", 110, "center", 0));

    layout_1.add("Speed", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Motor Controller");
        builder.addDoubleProperty("Value", sIntakeTilt::getSpeed, null);
      }
    }).withWidget(BuiltInWidgets.kMotorController)
      .withProperties(Map.of("orientation", "HORIZONTAL"));
  }

  // Intake wheels
  private void initLayout2() {
    ShuffleboardLayout layout_3 = test_tab.getLayout("Intake Wheels", BuiltInLayouts.kList)
      .withPosition(4, 0)
      .withSize(3, 4);

    layout_3.addBoolean("Inverted", IntakeWheels::isInverted)
      .withWidget(BuiltInWidgets.kBooleanBox);

    layout_3.add("Speed", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Motor Controller");
        builder.addDoubleProperty("Value", sIntakeWheels::getSpeed, null);
      }
    }).withWidget(BuiltInWidgets.kMotorController)
      .withProperties(Map.of("orientation", "HORIZONTAL"));
  }
}