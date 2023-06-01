package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OI;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.DeviceConstants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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
        -OI.driver_cntlr.getLeftY() * DrivetrainConstants.kSwerveMaxVel * DrivetrainConstants.kSpeedMult,
        -OI.driver_cntlr.getLeftX() * DrivetrainConstants.kSwerveMaxVel * DrivetrainConstants.kSpeedMult,
        -OI.driver_cntlr.getRightX() * 2*Math.PI * DrivetrainConstants.kTurnMult
      );
    }));
  }

  /** Updates the swerve speeds. Should be called every period. */
  public void updateSwerve() {m_swerve.update();}

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

  /** @return desired module states */
  public SwerveModuleState[] getDesiredStates() {return m_swerve.getDesiredStates();}

  public static void stop() {m_swerve.stopMotor();}

  /** @return an auto-balance command. The robot must be facing in a cardinal direction for this to properly work */
  public Command getBalanceCommand() {
    return run(
      new Runnable() {
        private double prevAngle = ((Math.abs(OI.pigeon.getYaw() % 180) - 90) <= 45) ? OI.pigeon.getRoll() : OI.pigeon.getPitch();

        public void run() {
          double angle = ((Math.abs(OI.pigeon.getYaw() % 180) - 90) <= 45) ? OI.pigeon.getRoll() : OI.pigeon.getPitch();

          if (angle > DrivetrainConstants.kBalanceTolerance && Math.abs(angle - prevAngle) < 3) {
            driveFieldRelativeVelocity(angle % 360 < 180 ? -DrivetrainConstants.kBalanceSpeed : DrivetrainConstants.kBalanceSpeed, 0, 0);
          } else {
            Drivetrain.stop();
          }
        }
      }
    );
  }
}