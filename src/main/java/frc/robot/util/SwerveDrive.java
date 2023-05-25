package frc.robot.util;

/** Controls a full set of {@link SwerveModule SwerveModules}. */
public class SwerveDrive {
  public final SwerveModule[] modules;

  public SwerveDrive(SwerveModule.SwerveModuleConstants consts_fl, SwerveModule.SwerveModuleConstants consts_fr, SwerveModule.SwerveModuleConstants consts_bl, SwerveModule.SwerveModuleConstants consts_br) {
    modules = new SwerveModule[] {
      new SwerveModule(consts_fl),
      new SwerveModule(consts_fr),
      new SwerveModule(consts_bl),
      new SwerveModule(consts_br)
    };
  }

  // Speed-based driving

  // Distance-based driving

  // Turning somehow??? how does one turn while moving??????? holonomic drive controller ftw????
}