package frc.robot.autos;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.util.SwerveDrive;

public class FollowPath {  
    PathPlannerTrajectory examplePath = PathPlanner.loadPath("Example Path", new PathConstraints(4, 3));
    SwerveDrive sSwerveDrive = new SwerveDrive(SwerveConstants.kSwerve_fl, SwerveConstants.kSwerve_fr, SwerveConstants.kSwerve_bl, SwerveConstants.kSwerve_br);

    FollowPathWithEvents command = new FollowPathWithEvents(
        sSwerveDrive.followTrajectoryCommand(examplePath, false),
        examplePath.getMarkers(),
        (HashMap<String, Command>)AutoConstants.eventMap
    );

    public Command getFollowCommand(){return command;}
}
