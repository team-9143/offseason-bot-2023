package frc.robot.util;

import frc.robot.Constants.PhysConstants;
import frc.robot.Constants.DrivetrainConstants;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.math.controller.PIDController;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

/** Controls a single swerve module. */
public class SwerveModule {
  public static class SwerveModuleConstants {
    public final byte drive_ID;
    public final byte angle_ID;
    public final byte cancoder_ID;
    public final Translation2d location;

    public final PIDController angle_controller;

    /**
     * @param drive_ID driving motor ID (brushless NEO)
     * @param angle_ID angular motor ID (brushless NEO)
     * @param cancoder_ID cancoder ID
     * @param location location of the wheel relative to the physical center of the robot (UNIT: meters)
     * @param angle_controller PID controller for the the module angle
     */
    public SwerveModuleConstants(int drive_ID, int angle_ID, int cancoder_ID, Translation2d location, PIDController angle_controller) {
      this.drive_ID = (byte) drive_ID;
      this.angle_ID = (byte) angle_ID;
      this.cancoder_ID = (byte) cancoder_ID;
      this.location = location;
      this.angle_controller = angle_controller;
    }
  }

  private final CANSparkMax drive_motor;
  private final CANSparkMax angle_motor;
  private final CANCoder cancoder;
  private final RelativeEncoder drive_encoder;

  private final PIDController angle_controller;

  protected SwerveModule(SwerveModuleConstants constants) {
    drive_motor = new CANSparkMax(constants.drive_ID, CANSparkMax.MotorType.kBrushless);
    angle_motor = new CANSparkMax(constants.angle_ID, CANSparkMax.MotorType.kBrushless);
    cancoder = new CANCoder(constants.cancoder_ID);
    drive_encoder = drive_motor.getEncoder();
    angle_controller = constants.angle_controller;

    cancoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    cancoder.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_20Ms);

    drive_encoder.setPositionConversionFactor(PhysConstants.kSwerveWheelGearbox * PhysConstants.kSwerveWheelCircumference); // UNIT: meters
    drive_encoder.setVelocityConversionFactor(PhysConstants.kSwerveWheelGearbox * PhysConstants.kSwerveWheelCircumference / 60); // UNIT: meters/s
    drive_encoder.setMeasurementPeriod(20);
    drive_encoder.setPosition(0);

    angle_controller.setIntegratorRange(-DrivetrainConstants.kSwerveMaxTurnVel, DrivetrainConstants.kSwerveMaxTurnVel);
    angle_controller.enableContinuousInput(-180, 180);
    angle_controller.setSetpoint(0);
  }

  /**
   * Set speed of the swerve module.
   *
   * @param speed module speed [-1.0..1.0]
   * @param rotation ccw angle (UNIT: degrees)
   */
  protected void drive(double speed, double angle) {
    drive_motor.set(speed);
    angle_motor.set(angle_controller.calculate(getAngle(), angle));
  }

  /**
   * Drive the swerve module based on a desired state.
   *
   * @param desired desired state
   */
  protected void desiredStateDrive(SwerveModuleState desired) {
    desired = SwerveModuleState.optimize(desired, Rotation2d.fromDegrees(getAngle()));
    // TODO: Use feedback controller for desired velocity
    drive(
      Math.max(-1, Math.min(desired.speedMetersPerSecond / DrivetrainConstants.kSwerveMaxVel, 1)),
      desired.angle.getDegrees()
    );
  }

  /** @return the angle of the module (UNIT: degrees) */
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

  public void stopMotor() {
    drive_motor.stopMotor();
    angle_motor.stopMotor();

    angle_controller.reset();
  }
}