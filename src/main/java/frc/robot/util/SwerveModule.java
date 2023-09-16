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
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

/** Controls a single swerve module. */
public class SwerveModule {
  public static class SwerveModuleConstants {
    public final byte drive_ID;
    public final byte angle_ID;
    public final byte cancoder_ID;
    public final double cancoderOffset;
    public final Translation2d location;

    public final PIDController speed_controller;
    public final PIDController angle_controller;

    /**
     * @param drive_ID driving motor ID (brushless NEO)
     * @param angle_ID angular motor ID (brushless NEO)
     * @param cancoder_ID cancoder ID
     * @param location location of the wheel relative to the physical center of the robot (forward, left) (UNIT: meters)
     * @param speed_controller PID controller to calculate motor speed from velocity error
     * @param angle_controller PID controller to calculate motor speed from degree error
     */
    public SwerveModuleConstants(int drive_ID, int angle_ID, int cancoder_ID, double cancoderOffset, Translation2d location, PIDController speed_controller, PIDController angle_controller) {
      this.drive_ID = (byte) drive_ID;
      this.angle_ID = (byte) angle_ID;
      this.cancoder_ID = (byte) cancoder_ID;
      this.cancoderOffset = cancoderOffset;
      this.location = location;
      this.speed_controller = speed_controller;
      this.angle_controller = angle_controller;
    }
  }

  private final CANSparkMax drive_motor;
  private final CANSparkMax angle_motor;
  private final CANCoder cancoder;
  private final double cancoderOffset;
  private final RelativeEncoder drive_encoder;

  private final PIDController speed_controller;
  private final PIDController angle_controller;

  protected SwerveModule(SwerveModuleConstants constants) {
    drive_motor = new CANSparkMax(constants.drive_ID, CANSparkMax.MotorType.kBrushless);
    angle_motor = new CANSparkMax(constants.angle_ID, CANSparkMax.MotorType.kBrushless);
    cancoder = new CANCoder(constants.cancoder_ID);
    cancoderOffset = constants.cancoderOffset;
    drive_encoder = drive_motor.getEncoder();
    speed_controller = constants.speed_controller;
    angle_controller = constants.angle_controller;

    // Set up cancoder
    cancoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360); // Set range [0..360]
    cancoder.configSensorDirection(false); // Set counterclockwise
    cancoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition); // Set absolute
    cancoder.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_20Ms);

    // Set up drive encoder units
    drive_encoder.setPositionConversionFactor(PhysConstants.kSwerveWheelGearbox * PhysConstants.kSwerveWheelCircumference); // UNIT: meters
    drive_encoder.setVelocityConversionFactor(PhysConstants.kSwerveWheelGearbox * PhysConstants.kSwerveWheelCircumference / 60); // UNIT: meters/s
    drive_encoder.setMeasurementPeriod(20);
    drive_encoder.setPosition(0);

    // Set up speed PID controller
    speed_controller.setIntegratorRange(-DrivetrainConstants.kMaxLinearVel, DrivetrainConstants.kMaxLinearVel);
    speed_controller.setSetpoint(0);

    // Set up rotational PID controller
    angle_controller.setIntegratorRange(-DrivetrainConstants.kSwerveRotateMaxSpd * 180/Math.PI, DrivetrainConstants.kSwerveRotateMaxSpd * 180/Math.PI);
    angle_controller.enableContinuousInput(-180, 180);
    angle_controller.setTolerance(DrivetrainConstants.kSwerveRotateTol);
    angle_controller.setSetpoint(0);
  }

  /**
   * Calculate, clamp, and set swerve module speed.
   *
   * @param speed module speed (UNIT: meters/s)
   * @param angle module angle (UNIT: ccw degrees)
   */
  protected void drive(double speed, double angle) {
    drive_motor.set(Math.max(-1, Math.min(1,
      speed_controller.calculate(getVelocity(), speed)
    )));

    var rotateSpeed = angle_controller.calculate(getAngle(), angle);
    if (angle_controller.atSetpoint()) {
      angle_motor.set(0);
    } else {
      angle_motor.set(Math.max(-DrivetrainConstants.kSwerveRotateMaxSpd, Math.min(DrivetrainConstants.kSwerveRotateMaxSpd,
        rotateSpeed
      )));
    }
  }

  /**
   * Drive the swerve module based on a desired angle and speed.
   *
   * @param desired desired state
   */
  protected void desiredStateDrive(SwerveModuleState desired) {
    desired = SwerveModuleState.optimize(desired, Rotation2d.fromDegrees(getAngle()));
    drive(
      desired.speedMetersPerSecond,
      desired.angle.getDegrees()
    );
  }

  /** @return the angle of the module (UNIT: ccw egrees) */
  public double getAngle() {
    return cancoder.getPosition() + cancoderOffset;
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
    speed_controller.reset();
  }
}