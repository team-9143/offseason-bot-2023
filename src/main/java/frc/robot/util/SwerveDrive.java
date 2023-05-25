package frc.robot.util;

import frc.robot.OI;
import frc.robot.Constants.DrivetrainConstants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

/** Controls a set of four {@link SwerveModule SwerveModules}. */
public class SwerveDrive {
  public final SwerveModule[] modules;
  private final HolonomicDriveController controller;
  private final SwerveDriveKinematics kinematics;
  private final SwerveDriveOdometry odometry;

  public SwerveDrive(SwerveModule.SwerveModuleConstants consts_fl, SwerveModule.SwerveModuleConstants consts_fr, SwerveModule.SwerveModuleConstants consts_bl, SwerveModule.SwerveModuleConstants consts_br) {
    modules = new SwerveModule[] {
      new SwerveModule(consts_fl),
      new SwerveModule(consts_fr),
      new SwerveModule(consts_bl),
      new SwerveModule(consts_br)
    };

    controller = new HolonomicDriveController(
      new PIDController(DrivetrainConstants.kXerrP, DrivetrainConstants.kXerrI, DrivetrainConstants.kXerrD),
      new PIDController(DrivetrainConstants.kYerrP, DrivetrainConstants.kYerrI, DrivetrainConstants.kYerrD),
      new ProfiledPIDController(DrivetrainConstants.kTerrP, DrivetrainConstants.kTerrI, DrivetrainConstants.kTerrD, new Constraints(DrivetrainConstants.kModuleTurnMaxVel, DrivetrainConstants.kModuleTurnMaxAccel))
    );

    kinematics = new SwerveDriveKinematics(
      consts_fl.location,
      consts_fr.location,
      consts_bl.location,
      consts_br.location
    );

    odometry = new SwerveDriveOdometry(kinematics, Rotation2d.fromDegrees(OI.pigeon.getYaw()), new SwerveModulePosition[] {
      new SwerveModulePosition(modules[0].getDistance(), Rotation2d.fromDegrees(modules[0].getAngle())),
      new SwerveModulePosition(modules[1].getDistance(), Rotation2d.fromDegrees(modules[1].getAngle())),
      new SwerveModulePosition(modules[2].getDistance(), Rotation2d.fromDegrees(modules[2].getAngle())),
      new SwerveModulePosition(modules[3].getDistance(), Rotation2d.fromDegrees(modules[3].getAngle()))
    });
  }

  /** Updates the drivetrain with current desired states. */
  public void update() {
    // TODO: update odometry and calculate motor speeds
  }

  /**
   * Sets desired module states.
   *
   * @param state_fl front left module state
   * @param state_fr front right module state
   * @param state_bl back left module state
   * @param state_br back right module state
   */
  public void setDesiredStates(SwerveModuleState state_fl, SwerveModuleState state_fr, SwerveModuleState state_bl, SwerveModuleState state_br) {
    state_fl = SwerveModuleState.optimize(state_fl, Rotation2d.fromDegrees(modules[0].getAngle()));
    state_fr = SwerveModuleState.optimize(state_fr, Rotation2d.fromDegrees(modules[1].getAngle()));
    state_bl = SwerveModuleState.optimize(state_bl, Rotation2d.fromDegrees(modules[2].getAngle()));
    state_br = SwerveModuleState.optimize(state_br, Rotation2d.fromDegrees(modules[3].getAngle()));
    // TODO: Implement somehow
  }

  /**
   * Sets desired module states based on desired velocity.
   *
   * @param forward forward speed (UNIT: meters/s)
   * @param left left speed (UNIT: meters/s)
   * @param ccw counter-clockwise speed (UNIT: degrees/s)
   */
  public void setDesiredVelocity(double forward, double left, double ccw) {
    var states = kinematics.toSwerveModuleStates(new ChassisSpeeds(forward, left, Math.toRadians(ccw)));
    setDesiredStates(states[0], states[1], states[2], states[3]);
  }

  // Distance-based driving

  // Turning somehow??? how does one turn while moving??????? holonomic drive controller ftw????

  public void stop() {
    for (SwerveModule module : modules) {module.stopMotor();}
  }
}