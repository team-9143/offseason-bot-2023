package frc.robot.util;

import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.OI;
import frc.robot.Constants.DrivetrainConstants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

/** Controls a set of four {@link SwerveModule SwerveModules}. Protected by {@link MotorSafety}. */
public class SwerveDrive extends MotorSafety {
  public final SwerveModule[] modules;
  private Pose2d desiredPose;
  private double desiredLinearVelocity; // UNIT: meters/s
  private SwerveModuleState[] desiredStates = new SwerveModuleState[] {
    new SwerveModuleState(),
    new SwerveModuleState(),
    new SwerveModuleState(),
    new SwerveModuleState()
  };

  /** {@code true} if being controlled by a desired location through a {@link HolonomicDriveController}. */
  private boolean locationControl = false;

  private final SwerveDriveKinematics kinematics;
  private final SwerveDrivePoseEstimator odometry; // If adding vision measurements, must initialize with relative pose instead of origin

  // Initialize PID controllers for position change -> velocity calculations
  private static final PIDController x_controller = new PIDController(DrivetrainConstants.kLinearP, DrivetrainConstants.kLinearI, DrivetrainConstants.kLinearD);
  private static final PIDController y_controller = new PIDController(DrivetrainConstants.kLinearP, DrivetrainConstants.kLinearI, DrivetrainConstants.kLinearD);
  private static final ProfiledPIDController theta_controller = new ProfiledPIDController(DrivetrainConstants.kAngularP, DrivetrainConstants.kAngularI, DrivetrainConstants.kAngularD, new Constraints(DrivetrainConstants.kMaxTurnVel, DrivetrainConstants.kMaxTurnAccel));
  static {
    x_controller.setIntegratorRange(-DrivetrainConstants.kMaxLinearVel, DrivetrainConstants.kMaxLinearVel);
    y_controller.setIntegratorRange(-DrivetrainConstants.kMaxLinearVel, DrivetrainConstants.kMaxLinearVel);
    theta_controller.setIntegratorRange(-DrivetrainConstants.kMaxTurnVel, DrivetrainConstants.kMaxTurnVel);
  }

  // Initiliaze holonomic controller for trajectory following
  private static final HolonomicDriveController m_controller = new HolonomicDriveController(x_controller, y_controller, theta_controller);
  static {
    m_controller.setTolerance(new Pose2d(DrivetrainConstants.kLinearPosTolerance, DrivetrainConstants.kLinearPosTolerance, Rotation2d.fromDegrees(DrivetrainConstants.kAngularPosTolerance)));
  }

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

    odometry = new SwerveDrivePoseEstimator(kinematics, Rotation2d.fromDegrees(OI.pigeon.getYaw()),
      new SwerveModulePosition[] {
        new SwerveModulePosition(modules[0].getDistance(), Rotation2d.fromDegrees(modules[0].getAngle())),
        new SwerveModulePosition(modules[1].getDistance(), Rotation2d.fromDegrees(modules[1].getAngle())),
        new SwerveModulePosition(modules[2].getDistance(), Rotation2d.fromDegrees(modules[2].getAngle())),
        new SwerveModulePosition(modules[3].getDistance(), Rotation2d.fromDegrees(modules[3].getAngle()))
      },
    new Pose2d());
  }

  /** Updates the drivetrain with desired speeds, and recalculates odometry. Should be called every robot loop. */
  public void update() {
    // Update odometry
    odometry.update(Rotation2d.fromDegrees(OI.pigeon.getYaw()), new SwerveModulePosition[] {
      new SwerveModulePosition(modules[0].getDistance(), Rotation2d.fromDegrees(modules[0].getAngle())),
      new SwerveModulePosition(modules[1].getDistance(), Rotation2d.fromDegrees(modules[1].getAngle())),
      new SwerveModulePosition(modules[2].getDistance(), Rotation2d.fromDegrees(modules[2].getAngle())),
      new SwerveModulePosition(modules[3].getDistance(), Rotation2d.fromDegrees(modules[3].getAngle()))
    });

    // If controlled by a desired location, calculates desired states through holonomic drive controller
    if (locationControl) {
      // As far as I can guess, rotation aspect of desired pose should be an angle pointing from robot to pose
      desiredStates = kinematics.toSwerveModuleStates(m_controller.calculate(
        odometry.getEstimatedPosition(),
        new Pose2d(desiredPose.getTranslation(), new Rotation2d(
          desiredPose.getX()-odometry.getEstimatedPosition().getX(),
          desiredPose.getY()-odometry.getEstimatedPosition().getY()
        )),
        desiredLinearVelocity,
        desiredPose.getRotation()
      ));
    }

    // Calculate and set speeds for swerve modules
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DrivetrainConstants.kMaxLinearVel);
    modules[0].desiredStateDrive(desiredStates[0]);
    modules[1].desiredStateDrive(desiredStates[1]);
    modules[2].desiredStateDrive(desiredStates[2]);
    modules[3].desiredStateDrive(desiredStates[3]);
  }

  /**
   * Sets desired module states, without optimizing.
   *
   * @param state_fl front left module state
   * @param state_fr front right module state
   * @param state_bl back left module state
   * @param state_br back right module state
   */
  public void setDesiredStates(SwerveModuleState state_fl, SwerveModuleState state_fr, SwerveModuleState state_bl, SwerveModuleState state_br) {
    desiredStates[0] = state_fl;
    desiredStates[1] = state_fr;
    desiredStates[2] = state_bl;
    desiredStates[3] = state_br;

    locationControl = false;

    feed();
  }

  /**
   * Sets desired module states based on field relative velocity.
   *
   * @param forward forward speed (UNIT: meters/s)
   * @param left left speed (UNIT: meters/s)
   * @param ccw counter-clockwise speed (UNIT: radians/s)
   */
  public void setDesiredVelocityFieldRelative(double forward, double left, double ccw) {
    var states = kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(forward, left, ccw, Rotation2d.fromDegrees(OI.pigeon.getYaw())));
    setDesiredStates(states[0], states[1], states[2], states[3]);
  }

  /**
   * Drive based on robot relative velocities.
   *
   * @param forward forward speed (UNIT: meters/s)
   * @param left left speed (UNIT: meters/s)
   * @param ccw counter-clockwise speed (UNIT: radians/s)
   */
  public void setDesiredVelocity(double forward, double left, double ccw) {
    var states = kinematics.toSwerveModuleStates(new ChassisSpeeds(forward, left, ccw));
    setDesiredStates(states[0], states[1], states[2], states[3]);
  }

  /**
   * Sets desired pose and linear velocity, to be controlled with a {@link HolonomicDriveController}
   *
   * @param desiredPose robot pose with the same origin as the odometry
   * @param desiredLinearVelocity desired linear velocity for feedforward calculation
   */
  public void setDesiredPose(Pose2d desiredPose, double desiredLinearVelocity) {
    this.desiredPose = desiredPose;
    this.desiredLinearVelocity = desiredLinearVelocity;

    if (!locationControl) {
      // Reset controllers if swapping into location control
      x_controller.reset();
      y_controller.reset();
      theta_controller.reset(odometry.getEstimatedPosition().getRotation().getRadians());
    }
    locationControl = true;

    feed();
  }

  /**
   * Reset the odometry to a given position.
   *
   * @param position robot position (UNIT: meters)
   */
  public void resetPosition(Pose2d position) {
    odometry.resetPosition(
      Rotation2d.fromDegrees(OI.pigeon.getYaw()),
      new SwerveModulePosition[] {
        new SwerveModulePosition(modules[0].getDistance(), Rotation2d.fromDegrees(modules[0].getAngle())),
        new SwerveModulePosition(modules[1].getDistance(), Rotation2d.fromDegrees(modules[1].getAngle())),
        new SwerveModulePosition(modules[2].getDistance(), Rotation2d.fromDegrees(modules[2].getAngle())),
        new SwerveModulePosition(modules[3].getDistance(), Rotation2d.fromDegrees(modules[3].getAngle()))
      },
      position
    );
  }

  /** @return the robot's current location */
  public Pose2d getPose() {return odometry.getEstimatedPosition();}

  /** @return {@code true} if location control is on and robot is near desired location */
  public boolean atReference() {return locationControl && m_controller.atReference();}

  /** @return the drivetrain's desired velocities */
  public ChassisSpeeds getDesiredSpeeds() {return kinematics.toChassisSpeeds(desiredStates);}

  /** @return the drivetrain's actual velocities, as measured by encoders */
  public ChassisSpeeds getActualSpeeds() {
    return kinematics.toChassisSpeeds(
      new SwerveModuleState(modules[0].getVelocity(), Rotation2d.fromDegrees(modules[0].getAngle())),
      new SwerveModuleState(modules[1].getVelocity(), Rotation2d.fromDegrees(modules[1].getAngle())),
      new SwerveModuleState(modules[2].getVelocity(), Rotation2d.fromDegrees(modules[2].getAngle())),
      new SwerveModuleState(modules[3].getVelocity(), Rotation2d.fromDegrees(modules[3].getAngle()))
    );
  }

  /** @return desired module states */
  public SwerveModuleState[] getDesiredStates() {return desiredStates;}

  /** Stop the modules and reset the desired states. */
  @Override
  public void stopMotor() {
    for (SwerveModule module : modules) {module.stopMotor();}

    setDesiredStates(new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState());
  }

  //   /**@return a command that follows a PathPlanner Trajectory */
  // public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
  //   return new SequentialCommandGroup(
  //     new InstantCommand(() -> {
  //       // Reset odometry for the first path you run during auto
  //       if(isFirstPath){
  //           this.resetPosition(traj.getInitialPose());
  //       }
  //     }),
  //       new PPSwerveControllerCommand(
  //           traj, 
  //           this::getPose, // Pose supplier
  //           this.kinematics, // SwerveDriveKinematics
  //           new PIDController(0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
  //           new PIDController(0, 0, 0), // Y controller (usually the same values as X controller)
  //           new PIDController(0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
  //           x -> this.getDesiredStates(), // Module states consumer
  //           true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
  //           this // Requires this drive subsystem
  //       )
  //   );
  // }

  @Override
  public String getDescription() {
    return "Swerve Drive";
  }
}