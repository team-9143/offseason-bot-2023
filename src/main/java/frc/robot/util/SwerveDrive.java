package frc.robot.util;

/** Controls a full set of {@link SwerveModule SwerveModules}. */
public class SwerveDrive {
  public final SwerveModule[] modules;

  public SwerveDrive(SwerveModule.SwerveModuleIDs consts_fl, SwerveModule.SwerveModuleIDs consts_fr, SwerveModule.SwerveModuleIDs consts_bl, SwerveModule.SwerveModuleIDs consts_br) {
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

  public void stop() {
    for (SwerveModule module : modules) {module.stopMotor();}
  }
}