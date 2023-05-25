package frc.robot.util;

import edu.wpi.first.wpilibj.MotorSafety;
import frc.robot.Constants.PhysConstants;
import edu.wpi.first.math.geometry.Translation2d;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

/** Controls a single swerve module. Protected by {@link MotorSafety}. */
public class SwerveModule extends MotorSafety {
  public static class SwerveModuleConstants {
    public final byte drive_ID;
    public final byte angle_ID;
    public final byte cancoder_ID;
    public final Translation2d location;

    /**
     * @param drive_ID driving motor ID (brushless NEO)
     * @param angle_ID angular motor ID (brushless NEO)
     * @param cancoder_ID cancoder ID
     * @param location location of the wheel relative to the physical center of the robot (UNIT: meters)
     */
    public SwerveModuleConstants(int drive_ID, int angle_ID, int cancoder_ID, Translation2d location) {
      this.drive_ID = (byte) drive_ID;
      this.angle_ID = (byte) angle_ID;
      this.cancoder_ID = (byte) cancoder_ID;
      this.location = location;
    }
  }

  private final CANSparkMax drive_motor;
  private final CANSparkMax angle_motor;
  private final CANCoder cancoder;
  private final RelativeEncoder drive_encoder;

  protected SwerveModule(SwerveModuleConstants constants) {
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

  /** @return the distance traveled by the module (UNIT: meters) */
  public double getDistance() {
    return drive_encoder.getPosition();
  }

  /** @return the velocity of the module (UNIT: meters/s) */
  public double getVelocity() {
    return drive_encoder.getVelocity();
  }

  @Override
  public void stopMotor() {
    drive_motor.stopMotor();
    angle_motor.stopMotor();
    feed();
    // TODO: implement PID disabling on stop
  }

  @Override
  public String getDescription() {return "Swerve Module";}
}