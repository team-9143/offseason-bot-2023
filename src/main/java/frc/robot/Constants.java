// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.util.SwerveModule.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Rotation2d;

// TODO: Tune/measure and fix all swerve constants
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
    public static final double kTiltGearbox = 1/35.0;
    public static final double kWheelGearbox = 1/3.0;

    public static final double kSwerveWheelGearbox = 1/5.14; // SDS L4 modules
    public static final double kSwerveWheelCircumference = 0.1016 * Math.PI; // UNIT: meters
  }

  public static class DeviceConstants {
    public static final byte kDriverCntlrPort = 0;
    public static final byte kOperatorCntlrPort = 1;
    public static final byte
      kPigeonID = 5,
      kIntakeWheelsID = 6,
      kIntakeTiltRightID = 7,
      kIntakeTiltLeftID = 8;
  }

  public static class DrivetrainConstants {
    // Applies to all teleop driving
    public static final double kSpeedMult = 1;
    public static final double kTurnMult = 0.7;

    // Charge station balancing
    public static final double kBalanceTolerance = 2; // UNIT: degrees
    public static final double kBalanceVel = 1; // UNIT: meters/s

    // TODO: Measure maximum drivetrain velocities
    public static final double kMaxSwerveRotate = 0.25; // Maximum rotational motor speed
    public static final double kMaxLinearVel = 14; // UNIT: meters/s
    public static final double kMaxTurnVel = 10; // UNIT: radians/s
    public static final double kMaxTurnAccel = kMaxTurnVel * 4; // UNIT: radians/s/s

    // Controllers for drivetrain position error -> velocity
    public static final double
      kLinearP = 0.7,
      kLinearI = 0.4,
      kLinearD = 0.5;
    public static final double
      kAngularP = 0.637,
      kAngularI = 0.2,
      kAngularD = 0.3;

    // Drivetrain location control tolerance
    public static final double kLinearPosTolerance = 0.0127; // UNIT: meters
    public static final double kAngularPosTolerance = 0.75; // UNIT: degrees
  }

  public static class SwerveConstants {
    public static final SwerveModuleConstants
      kSwerve_fl = new SwerveModuleConstants(
        41, 42, 43, 0.325,
        new Translation2d(0.22225, 0.22225),
        new PIDController(0.07, 0, 0),
        new PIDController(0.001, 0, 0)
      ),
      kSwerve_fr = new SwerveModuleConstants(
        11, 12, 13, -2.285,
        new Translation2d(0.22225, -0.22225),
        new PIDController(0.07, 0, 0),
        new PIDController(0.001, 0, 0)
      ),
      kSwerve_bl = new SwerveModuleConstants(
        31, 32, 33, -5.713,
        new Translation2d(-0.22225, 0.22225),
        new PIDController(0.07, 0, 0),
        new PIDController(0.001, 0, 0)
      ),
      kSwerve_br = new SwerveModuleConstants(
        21, 22, 23, -3.076,
        new Translation2d(-0.22225, -0.22225),
        new PIDController(0.07, 0, 0),
        new PIDController(0.001, 0, 0)
      );

    public static final SwerveModuleState[] xStanceStates = new SwerveModuleState[] {
      new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
      new SwerveModuleState(0, Rotation2d.fromDegrees(135)),
      new SwerveModuleState(0, Rotation2d.fromDegrees(135)),
      new SwerveModuleState(0, Rotation2d.fromDegrees(45))
    };
  }

  public static class IntakeConstants {
    public static final double kTiltMaxSpeed = 0.45;

    // Delay to shoot/spit a game piece
    public static final double kShootTimer = 0.5;

    // Non-PID intake movement
    public static final double
      kUpSpeed = PhysConstants.kTiltGearbox * -3.5,
      kDownSpeed = PhysConstants.kTiltGearbox * 2.8,
      kSteadySpeed = PhysConstants.kTiltGearbox * -0.35;

    // Preset positions and tolerances (UNIT: degrees)
    public static final double
      kUpPos = 1.08,
      kMidPos = 36,
      kDownPos = 104.4;
    public static final double
      kUpPosTolerance = -9, // Check as error > tolerance
      kMidPosTolerance = 1.5, // Check as abs(error) > tolerance
      kDownPosTolerance = 2; // Check as error < tolerance

    // Intake tilt PID gains
    public static final double
      kDownP = PhysConstants.kTiltGearbox * 0.0806,
      kDownI = PhysConstants.kTiltGearbox * 0.0612,
      kDownD = PhysConstants.kTiltGearbox * 0.0305;
    public static final double
      kUpP = PhysConstants.kTiltGearbox * 0.0862,
      kUpI = PhysConstants.kTiltGearbox * 0.0584,
      kUpD = PhysConstants.kTiltGearbox * 0.0277;
    public static final double
      kSteadyP = PhysConstants.kTiltGearbox * 0.0917,
      kSteadyI = PhysConstants.kTiltGearbox * 0.0639,
      kSteadyD = PhysConstants.kTiltGearbox * 0.0194;
  }
}