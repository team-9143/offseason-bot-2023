package frc.robot.util;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/** Controls a full set of {@link SwerveModule SwerveModules}. */
public class SwerveDrive {
  public final SwerveModule[] modules;
  private final SwerveDriveKinematics kinematics;

  public SwerveDrive(SwerveModule.SwerveModuleConstants consts_fl, SwerveModule.SwerveModuleConstants consts_fr, SwerveModule.SwerveModuleConstants consts_bl, SwerveModule.SwerveModuleConstants consts_br) {
    modules = new SwerveModule[] {
      new SwerveModule(consts_fl),
      new SwerveModule(consts_fr),
      new SwerveModule(consts_bl),
      new SwerveModule(consts_br)
    };

    kinematics = new SwerveDriveKinematics(
      consts_fl.location,
      consts_fr.location,
      consts_bl.location,
      consts_br.location
    );
  }

  // Speed-based driving

  // Distance-based driving

  // Turning somehow??? how does one turn while moving??????? holonomic drive controller ftw????

  public void stop() {
    for (SwerveModule module : modules) {module.stopMotor();}
  }
}