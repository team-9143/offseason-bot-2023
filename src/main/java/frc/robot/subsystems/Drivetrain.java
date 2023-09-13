package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OI;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.SwerveConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.wpilibj2.command.Command;

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
    SwerveConstants.kSwerve_fl,
    SwerveConstants.kSwerve_fr,
    SwerveConstants.kSwerve_bl,
    SwerveConstants.kSwerve_br
  );

  private Drivetrain() {
    setDefaultCommand(run(() -> {
      m_swerve.setDesiredVelocityFieldRelative(
        -OI.driver_cntlr.getLeftY() * DrivetrainConstants.kSwerveMaxVel * DrivetrainConstants.kSpeedMult,
        -OI.driver_cntlr.getLeftX() * DrivetrainConstants.kSwerveMaxVel * DrivetrainConstants.kSpeedMult,
        -OI.driver_cntlr.getRightX() * 2*Math.PI * DrivetrainConstants.kTurnMult
      );
    }));
  }

  /** Updates the swerve speeds and odometry. Should be called every period. */
  public void updateSwerve() {m_swerve.update();}

  /**
   * Drive based on field relative velocities. Must be continuously called.
   *
   * @param forward forward speed (UNIT: meters/s)
   * @param left left speed (UNIT: meters/s)
   * @param ccw counter-clockwise speed (UNIT: radians/s)
   */
  public void driveFieldRelativeVelocity(double forward, double left, double ccw) {
    m_swerve.setDesiredVelocityFieldRelative(forward, left, ccw);
  }

  /**
   * Drive based on robot relative velocities. Must be continuously called.
   *
   * @param forward forward speed (UNIT: meters/s)
   * @param left left speed (UNIT: meters/s)
   * @param ccw counter-clockwise speed (UNIT: radians/s)
   */
  public void driveRobotRelativeVelocity(double forward, double left, double ccw) {
    m_swerve.setDesiredVelocity(forward, left, ccw);
  }

  /**
   * Drive to a position, relative to the robot's starting position. Must be continuously called.
   *
   * @param forward forward distance (UNIT: meters)
   * @param left left distance (UNIT: meters)
   * @param ccw counter-clockwise angle (UNIT: degrees)
   * @param FFspd desired linear velocity for feedforward calculation (UNIT: meters/s)
   */
  public void driveToLocation(double forward, double left, double ccw, double FFspd) {
    m_swerve.setDesiredPose(new Pose2d(forward, left, Rotation2d.fromDegrees(ccw)), FFspd);
  }

  /** Set the drivetrain to x-stance. Must be continuously called. */
  public void toXStance() {
    m_swerve.setDesiredStates(
      SwerveConstants.xStanceStates[0],
      SwerveConstants.xStanceStates[1],
      SwerveConstants.xStanceStates[2],
      SwerveConstants.xStanceStates[3]
    );
  }

  /**
   * Reset the odometry to a given position.
   *
   * @param forward forward distance (UNIT: meters)
   * @param left left distance (UNIT: meters)
   * @param ccw counter-clockwise angle (UNIT: degrees)
   */
  public void resetPosition(double forward, double left, double ccw) {
    m_swerve.resetPosition(new Pose2d(forward, left, Rotation2d.fromDegrees(ccw)));
  }

  /** @return the estimated pose of the robot */
  public Pose2d getPose() {return m_swerve.getPose();}

  /** @return {@code true} if location control is on and robot is near desired location */
  public boolean atReference() {return m_swerve.atReference();}

  /** @return the drivetrain's desired velocities */
  public ChassisSpeeds getDesiredSpeeds() {return m_swerve.getDesiredSpeeds();}
  /** @return the drivetrain's actual velocities, as measured by encoders */
  public ChassisSpeeds getActualSpeeds() {return m_swerve.getActualSpeeds();}

  /** @return desired module states */
  public SwerveModuleState[] getDesiredStates() {return m_swerve.getDesiredStates();}

  public static void stop() {m_swerve.stopMotor();}

  /** @return an auto-balance command. Works in any robot orientation */
  public Command getBalanceCommand() {
    return run(() -> {
        double angle = Math.hypot(OI.pigeon.getPitch(), OI.pigeon.getRoll());

        if (angle > DrivetrainConstants.kBalanceTolerance) {
          // Calculate field-relative direction of movement based on robot orientation
          double sign;
          double yaw = MathUtil.inputModulus(OI.pigeon.getYaw(), 0, 360);
          if (yaw <= 90) {
            // 0-90: negative pitch, negative roll
            sign = (OI.pigeon.getPitch() > DrivetrainConstants.kBalanceTolerance) ? -OI.pigeon.getPitch() : -OI.pigeon.getRoll();
          } else if (yaw <= 180) {
            // 90-180: positive pitch, negative roll
            sign = (OI.pigeon.getPitch() > DrivetrainConstants.kBalanceTolerance) ? OI.pigeon.getPitch() : -OI.pigeon.getRoll();
          } else if (yaw <= 270) {
            // 180-270: positive pitch, positive roll
            sign = (OI.pigeon.getPitch() > DrivetrainConstants.kBalanceTolerance) ? OI.pigeon.getPitch() : OI.pigeon.getRoll();
          } else {
            // 270-360: negative pitch, positive roll
            sign = (OI.pigeon.getPitch() > DrivetrainConstants.kBalanceTolerance) ? -OI.pigeon.getPitch() : OI.pigeon.getRoll();
          }

          driveFieldRelativeVelocity(Math.copySign(DrivetrainConstants.kBalanceVel, sign), 0, 0);
        } else {
          stop();
        }
      }
    );
  }
}