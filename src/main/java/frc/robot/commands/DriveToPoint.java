package frc.robot.commands;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;


public class DriveToPoint {
    
    PathConstraints constraints;
    
    public DriveToPoint(){

        constraints = new PathConstraints(3.0, 4.0,
        Units.degreesToRadians(540), Units.degreesToRadians(720));

        
    }
    
    public Command cmd(PathPlannerPath path){
    
    Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
    path,
        constraints
        // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
    );
    return pathfindingCommand;
    }


}
