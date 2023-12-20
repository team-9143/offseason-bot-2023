package frc.robot.util;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;

// Pathplanner 2024

// import com.pathplanner.lib.path.PathPlannerPath;
// import com.pathplanner.lib.commands.FollowPathHolonomic;
// import com.pathplanner.lib.util.PIDConstants;
// import com.pathplanner.lib.util.ReplanningConfig;

// TODO: Setup complex paths with commands
/** Utility class to provide pathing commands. */
public class Pathing {
  /**
   * Utility method to follow a Pathplanner path.
   *
   * @param pathName name of the path, ".path" excluded (should be under "{deploy}/pathplanner/")
   * @return a command for Pathplanner path following
   */
  public static Command getFollowPathCommand(String pathName) {
    return new PPSwerveControllerCommand(
      PathPlanner.loadPath(pathName, Constants.DrivetrainConstants.kMaxLinearVel, Constants.DrivetrainConstants.kMaxLinearAccel), // Path and constraints to follow
      Drivetrain.getInstance()::getPose, // Pose supplier (field relative)
      new PIDController(Constants.DrivetrainConstants.kLinearP.getAsDouble(), Constants.DrivetrainConstants.kLinearI.getAsDouble(), Constants.DrivetrainConstants.kLinearD.getAsDouble()), // Linear PID controller
      new PIDController(Constants.DrivetrainConstants.kLinearP.getAsDouble(), Constants.DrivetrainConstants.kLinearI.getAsDouble(), Constants.DrivetrainConstants.kLinearD.getAsDouble()), // Linear PID controller
      new PIDController(Constants.DrivetrainConstants.kAngularP.getAsDouble(), Constants.DrivetrainConstants.kAngularI.getAsDouble(), Constants.DrivetrainConstants.kAngularD.getAsDouble()), // Rotational PID controller
      speeds -> Drivetrain.getInstance().driveFieldRelativeVelocity(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond), // Velocities consumer (field relative),
      false, // Whether to mirror path depending on alliance color
      Drivetrain.getInstance() // Subsystem req's
    );

    // Pathplanner 2024

    // return new FollowPathHolonomic(
    //   PathPlannerPath.fromPathFile(pathName), // Path to follow
    //   Drivetrain.getInstance()::getPose, // Pose supplier (field relative)
    //   Drivetrain.getInstance()::getMeasuredSpeeds, // Velocities supplier (robot relative)
    //   speeds -> Drivetrain.getInstance().driveRobotRelativeVelocity(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond), // Velocities consumer (robot relative)
    //   new PIDConstants(Constants.DrivetrainConstants.kLinearP.getAsDouble(), Constants.DrivetrainConstants.kLinearI.getAsDouble(), Constants.DrivetrainConstants.kLinearD.getAsDouble()), // Translational PID controller constants
    //   new PIDConstants(Constants.DrivetrainConstants.kAngularP.getAsDouble(), Constants.DrivetrainConstants.kAngularI.getAsDouble(), Constants.DrivetrainConstants.kAngularD.getAsDouble()), // Rotational PID controller constants
    //   Constants.DrivetrainConstants.kMaxLinearVel, // Maximum drive speed
    //   Math.sqrt(2 * Math.pow(Constants.SwerveConstants.kSwerve_fl.location.getX(), 2)), // Distance from center of rotation to "farthest" module
    //   new ReplanningConfig(false, false),
    //   Drivetrain.getInstance() // Subsystem req's
    // );
  }

  /**
   * Utility method that returns a command to drive to a position, based on the odometry.
   *
   * @param forward forward distance (UNIT: meters)
   * @param left left distance (UNIT: meters)
   * @param ccw ccw-positive angle (UNIT: degrees)
   * @return a command that ends when the robot is at the given position
   */
  public static Command getMoveCommand(double forward, double left, double ccw) {
    return new FunctionalCommand(
      () -> {},
      () -> Drivetrain.getInstance().driveToLocation(forward, left, ccw),
      interrupted -> Drivetrain.stop(),
      Drivetrain.getInstance()::atReference,
      Drivetrain.getInstance()
    );
  }
}