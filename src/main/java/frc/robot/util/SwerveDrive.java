package frc.robot.util;

public class SwerveDrive {
  public final SwerveModule
    module_fl,
    module_fr,
    module_bl,
    module_br;

  public SwerveDrive(SwerveModule.SwerveModuleConstants module_fl, SwerveModule.SwerveModuleConstants module_fr, SwerveModule.SwerveModuleConstants module_bl, SwerveModule.SwerveModuleConstants module_br) {
    this.module_fl = new SwerveModule(module_fl);
    this.module_fr = new SwerveModule(module_fr);
    this.module_bl = new SwerveModule(module_bl);
    this.module_br = new SwerveModule(module_br);
  }

  // Speed-based driving

  // Distance-based driving

  // Turning somehow??? how does one turn while moving??????? holonomic drive controller ftw????
}