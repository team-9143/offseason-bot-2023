package frc.robot.autos;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Drivetrain;

public class FollowPath {  
    PathPlannerTrajectory examplePath = PathPlanner.loadPath("Example Path", new PathConstraints(4, 3));

    FollowPathWithEvents command = new FollowPathWithEvents(
        Drivetrain.getInstance().followTrajectoryCommand(examplePath, false),
        examplePath.getMarkers(),
        (HashMap<String, Command>)AutoConstants.eventMap
    );
}
