package frc.robot.util;

import frc.robot.Constants.DrivetrainConstants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

/** Controls a set of four {@link SwerveModule SwerveModules}. */
public class SwerveDrive {
  public final SwerveModule[] modules;
  private final HolonomicDriveController controller;
  private final SwerveDriveKinematics kinematics;

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
  }

  public void desiredStateDrive(SwerveModuleState state_fl, SwerveModuleState state_fr, SwerveModuleState state_bl, SwerveModuleState state_br) {
    // TODO: Implement by using PID controllers in swerve modules
  }

  /**
   * Velocity-based driving.
   *
   * @param forward forward speed (UNIT: meters/s)
   * @param left left speed (UNIT: meters/s)
   * @param ccw counter-clockwise speed (UNIT: degrees/s)
   */
  public void velocityDrive(double forward, double left, double ccw) {
    var states = kinematics.toSwerveModuleStates(new ChassisSpeeds(forward, left, Math.toRadians(ccw)));
    desiredStateDrive(states[0], states[1], states[2], states[3]);
  }

  // Distance-based driving

  // Turning somehow??? how does one turn while moving??????? holonomic drive controller ftw????

  public void stop() {
    for (SwerveModule module : modules) {module.stopMotor();}
  }
}