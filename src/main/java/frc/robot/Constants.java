// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import frc.robot.util.SwerveModule.SwerveModuleConstants;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.TunableNumber;
import edu.wpi.first.math.controller.PIDController;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class PhysConstants {
    public static final double kSwerveWheelGearbox = 1/5.14; // SDS L4 modules
    public static final double kSwerveWheelCircumference = 0.09779 * Math.PI; // UNIT: meters
  }

  public static class DeviceConstants {
    public static final byte kDriverCntlrPort = 0;
    public static final byte kOperatorCntlrPort = 1;
    public static final byte
      kPigeonID = 5;
  }

  public static class DrivetrainConstants {
    // Applies to all teleop driving
    public static final double kSpeedMult = 1;
    public static final double kTurnMult = 0.7;

    // Charge station balancing
    public static final double kBalanceTolerance = 2; // UNIT: degrees
    public static final double kBalanceVel = 1; // UNIT: meters/s

    // TODO: Adjust maximum swerve rotational speed
    // Module wheel rotation
    public static final double kSwerveRotateMaxSpeed = 0.65; // Maximum rotational motor speed

    // TODO: Measure maximum drivetrain velocities
    // Upper bound drivetrain velocities
    public static final double kMaxLinearVel = 5.5; // 0.5 m/s decrease to account for friction (UNIT: meters/s)
    /** At 5.5 ms/s, maximum rotational velocity is just under 3.95 rotations/s, decreased to 1.5 rotations/s to allow for lateral speed while turning */
    public static final double kMaxTurnVel = 9.5; // UNIT: radians/s
    public static final double kMaxTurnAccel = kMaxTurnVel * 2; // UNIT: radians/s/s

    // TODO: Tune drivetrain position PID gains
    // Gains for drivetrain position error -> velocity
    public static final TunableNumber
      kLinearP = new TunableNumber("P", 0.7, "Linear Positioning"),
      kLinearI = new TunableNumber("I", 0, "Linear Positioning"),
      kLinearD = new TunableNumber("D", 0, "Linear Positioning");
    public static final TunableNumber
      kAngularP = new TunableNumber("P", 0.637, "Angular Positioning"),
      kAngularI = new TunableNumber("I", 0, "Angular Positioning"),
      kAngularD = new TunableNumber("D", 0, "Angular Positioning");

    // TODO: Decide on drivetrain pose tolerance
    // Drivetrain location control tolerance
    public static final Pose2d kPosTolerance = new Pose2d(
      new Translation2d(0.0127, 0.0127), // UNIT: meters
      Rotation2d.fromDegrees(0.75)
    );
  }

  public static class SwerveConstants {
    // TODO: Make sure cancoder magnetic range is within proper bounds (CANCoder.configMagnetOffset())
    // TODO: Tune PID gains for swerve module angle and velocity error
    public static final TunableNumber
      kDriveP = new TunableNumber("P", 1.5e-2, "Module Drive"),
      kDriveD = new TunableNumber("D", 2.3e-3, "Module Drive");
    public static final TunableNumber
      kAngleP = new TunableNumber("P", 0.0065, "Module Angle"),
      kAngleD = new TunableNumber("D", 0.00005, "Module Angle");

    public static final SwerveModuleConstants
      kSwerve_fl = new SwerveModuleConstants(
        41, 42, 43, 261.229,
        new Translation2d(0.22225, 0.22225),
        new PIDController(kDriveP.getAsDouble(), 0, kDriveD.getAsDouble()),
        new PIDController(kAngleP.getAsDouble(), 0, kAngleD.getAsDouble())
      ),
      kSwerve_fr = new SwerveModuleConstants(
        11, 12, 13, -150.029,
        new Translation2d(0.22225, -0.22225),
        new PIDController(kDriveP.getAsDouble(), 0, kDriveD.getAsDouble()),
        new PIDController(kAngleP.getAsDouble(), 0, kAngleD.getAsDouble())
      ),
      kSwerve_bl = new SwerveModuleConstants(
        31, 32, 33, 301.465,
        new Translation2d(-0.22225, 0.22225),
        new PIDController(kDriveP.getAsDouble(), 0, kDriveD.getAsDouble()),
        new PIDController(kAngleP.getAsDouble(), 0, kAngleD.getAsDouble())
      ),
      kSwerve_br = new SwerveModuleConstants(
        21, 22, 23, -3.428,
        new Translation2d(-0.22225, -0.22225),
        new PIDController(kDriveP.getAsDouble(), 0, kDriveD.getAsDouble()),
        new PIDController(kAngleP.getAsDouble(), 0, kAngleD.getAsDouble())
      );

      static {
        kDriveP.bindTo(val -> {
          kSwerve_fl.speed_controller.setP(val);
          kSwerve_fr.speed_controller.setP(val);
          kSwerve_bl.speed_controller.setP(val);
          kSwerve_br.speed_controller.setP(val);
        });
        kDriveD.bindTo(val -> {
          kSwerve_fl.speed_controller.setD(val);
          kSwerve_fr.speed_controller.setD(val);
          kSwerve_bl.speed_controller.setD(val);
          kSwerve_br.speed_controller.setD(val);
        });

        kAngleP.bindTo(val -> {
          kSwerve_fl.angle_controller.setP(val);
          kSwerve_fr.angle_controller.setP(val);
          kSwerve_bl.angle_controller.setP(val);
          kSwerve_br.angle_controller.setP(val);
        });
        kAngleD.bindTo(val -> {
          kSwerve_fl.angle_controller.setD(val);
          kSwerve_fr.angle_controller.setD(val);
          kSwerve_bl.angle_controller.setD(val);
          kSwerve_br.angle_controller.setD(val);
        });
      }
  }
}