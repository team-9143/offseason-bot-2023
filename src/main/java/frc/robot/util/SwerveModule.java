package frc.robot.util;

import edu.wpi.first.wpilibj.MotorSafety;
import frc.robot.Constants.PhysConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

/** Controls a single swerve module. Protected by {@link MotorSafety}. */
public class SwerveModule extends MotorSafety {
  public static class SwerveModuleIDs {
    public final byte drive_ID;
    public final byte angle_ID;
    public final byte cancoder_ID;

    /**
     * @param drive_ID driving motor ID (brushless NEO)
     * @param angle_ID angular motor ID (brushless NEO)
     * @param cancoder_ID cancoder ID
     */
    public SwerveModuleIDs(int drive_ID, int angle_ID, int cancoder_ID) {
      this.drive_ID = (byte) drive_ID;
      this.angle_ID = (byte) angle_ID;
      this.cancoder_ID = (byte) cancoder_ID;
    }
  }

  private final CANSparkMax drive_motor;
  private final CANSparkMax angle_motor;
  private final CANCoder cancoder;
  private final RelativeEncoder drive_encoder;

  protected SwerveModule(SwerveModuleIDs constants) {
    drive_motor = new CANSparkMax(constants.drive_ID, CANSparkMax.MotorType.kBrushless);
    angle_motor = new CANSparkMax(constants.angle_ID, CANSparkMax.MotorType.kBrushless);
    cancoder = new CANCoder(constants.cancoder_ID);
    drive_encoder = drive_motor.getEncoder();

    cancoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    cancoder.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_20Ms);

    drive_encoder.setPositionConversionFactor(PhysConstants.kSwerveWheelGearbox * PhysConstants.kSwerveWheelCircumference); // UNIT: meters
    drive_encoder.setVelocityConversionFactor(PhysConstants.kSwerveWheelGearbox * PhysConstants.kSwerveWheelCircumference / 60); // UNIT: meters/s
    drive_encoder.setMeasurementPeriod(20);
    drive_encoder.setPosition(0);
  }

  /**
   * Set speed of the swerve module.
   *
   * @param speed module speed [-1.0..1.0]
   * @param rotation rotation speed [-1.0..1.0]
   */
  protected void drive(double speed, double rotation) {
    drive_motor.set(speed);
    angle_motor.set(rotation);
    feed();
  }

  /** @return the angle of the module */
  public double getAngle() {
    return cancoder.getPosition();
  }

  /** @return the distance traveled by the module (UNIT: inches) */
  public double getDistance() {
    return drive_encoder.getPosition();
  }

  /** @return the velocity of the module (UNIT: inches/s) */
  public double getVelocity() {
    return drive_encoder.getVelocity();
  }

  @Override
  public void stopMotor() {
    drive_motor.stopMotor();
    angle_motor.stopMotor();
    feed();
  }

  @Override
  public String getDescription() {return "Swerve Module";}
}