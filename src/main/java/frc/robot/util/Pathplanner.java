package frc.robot.util;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

// TODO: Setup complex paths with commands
public class Pathplanner {
  /**
   * @param pathName name of the path, ".path" excluded
   * @return a command for path following
   */
  public static Command getFollowPathCommand(String pathName) {
    return new FollowPathHolonomic(
      PathPlannerPath.fromPathFile(pathName), // Path to follow
      Drivetrain.getInstance()::getPose, // Pose supplier (field relative)
      Drivetrain.getInstance()::getMeasuredSpeeds, // Velocities supplier (robot relative)
      speeds -> Drivetrain.getInstance().driveRobotRelativeVelocity(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond), // Velocities consumer (robot relative)
      new HolonomicPathFollowerConfig(
        new PIDConstants(Constants.DrivetrainConstants.kLinearP.getAsDouble(), Constants.DrivetrainConstants.kLinearI.getAsDouble(), Constants.DrivetrainConstants.kLinearD.getAsDouble()), // Translational PID controller constants
        new PIDConstants(Constants.DrivetrainConstants.kAngularP.getAsDouble(), Constants.DrivetrainConstants.kAngularI.getAsDouble(), Constants.DrivetrainConstants.kAngularD.getAsDouble()), // Rotational PID controller constants
        Constants.DrivetrainConstants.kMaxLinearVel, // Maximum drive speed
        Math.sqrt(2 * Math.pow(Constants.SwerveConstants.kSwerve_fl.location.getX(), 2)), // Distance from center of rotation to "farthest" module
        new ReplanningConfig(false, false)
      ),
      Drivetrain.getInstance() // Subsystem req's
    );
  }
}