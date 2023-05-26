package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OI;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.DeviceConstants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.wpilibj2.command.Command;
import java.lang.Runnable;

import frc.robot.util.SwerveDrive;

/** Controls the robot drivetrain. */
public class Drivetrain extends SubsystemBase {
  private static Drivetrain m_instance;

  /** @return the singleton instance */
  public static synchronized Drivetrain getInstance() {
    if (m_instance == null) {
      m_instance = new Drivetrain();
    }
    return m_instance;
  }

  public static final SwerveDrive m_swerve = new SwerveDrive(
    DeviceConstants.kSwerve_fl,
    DeviceConstants.kSwerve_fr,
    DeviceConstants.kSwerve_bl,
    DeviceConstants.kSwerve_br
  );

  private Drivetrain() {
    setDefaultCommand(run(() -> {
      m_swerve.setDesiredVelocityFieldRelative(
        -OI.driver_cntlr.getLeftY() * DrivetrainConstants.kModuleWheelMaxVel * DrivetrainConstants.kSpeedMult,
        -OI.driver_cntlr.getLeftX() * DrivetrainConstants.kModuleWheelMaxVel * DrivetrainConstants.kSpeedMult,
        -OI.driver_cntlr.getRightX() * Math.PI
      );
    }));
  }

  @Override
  public void periodic() {
    // TODO: Fix lag by potentially moving update to robot periodic
    // Subsystem periodic methods are called before command executions, so there is a 20 ms lag
    m_swerve.update();
  }

  /**
   * Drive based on field relative velocities.
   *
   * @param forward forward speed (UNIT: meters/s)
   * @param left left speed (UNIT: meters/s)
   * @param ccw counter-clockwise speed (UNIT: radians/s)
   */
  public void driveFieldRelativeVelocity(double forward, double left, double ccw) {
    m_swerve.setDesiredVelocityFieldRelative(forward, left, ccw);
  }

  /**
   * Drive based on robot relative velocities.
   *
   * @param forward forward speed (UNIT: meters/s)
   * @param left left speed (UNIT: meters/s)
   * @param ccw counter-clockwise speed (UNIT: radians/s)
   */
  public void driveRobotRelativeVelocity(double forward, double left, double ccw) {
    m_swerve.setDesiredVelocity(forward, left, ccw);
  }

  /**
   * Drive to a position, relative to the robot's starting position.
   *
   * @param forward forward distance (UNIT: meters)
   * @param left left distance (UNIT: meters)
   * @param ccw counter-clockwise angle (UNIT: degrees)
   * @param FFspd desired linear velocity for feedforward calculation (UNIT: meters/s)
   */
  public void driveToLocation(double forward, double left, double ccw, double FFspd) {
    m_swerve.setDesiredPose(new Pose2d(forward, left, Rotation2d.fromDegrees(ccw)), FFspd);
  }

  /** @return the estimated pose of the robot */
  public Pose2d getPose() {return m_swerve.getPose();}

  /** @return {@code true} if location control is on and robot is near desired location */
  public boolean atReference() {return m_swerve.atReference();}

  /** @return the drivetrain's desired velocities */
  public ChassisSpeeds getDesiredSpeeds() {return m_swerve.getDesiredSpeeds();}
  /** @return the drivetrain's actual velocities, as measured by encoders */
  public ChassisSpeeds getActualSpeeds() {return m_swerve.getActualSpeeds();}

  public static void stop() {m_swerve.stopMotor();}

  /** @return an auto-balance command */
  public Command getBalanceCommand() {
    return runEnd(
      new Runnable() {
        private double previousPitch = -OI.pigeon.getPitch();

        public void run() {
          // Pitch should increase to the back
          double pitch = -OI.pigeon.getPitch();

          if (Math.abs(pitch) > DrivetrainConstants.kBalanceTolerance && Math.abs(pitch - previousPitch) < 3) {
            driveRobotRelativeVelocity(Math.copySign(DrivetrainConstants.kBalanceSpeed, pitch), 0, 0);
          } else {
            // Stop movement on a large pitch change (usually denoting a fall) or when stabilized
            stop();
          }

          previousPitch = pitch;
        }
      },
      Drivetrain::stop
    );
  }
}