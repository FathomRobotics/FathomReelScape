package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotState;
import frc.robot.subsystems.Claw.Claw;
import frc.robot.subsystems.Claw.ClawStates;
import frc.robot.subsystems.ClawPivot.ClawPivot;
import frc.robot.subsystems.ClawWrist.ClawWrist;
import frc.robot.subsystems.Elevator.Elevator;

public class PrepCoralScoreCommand extends SequentialCommandGroup {
    public PrepCoralScoreCommand(Claw claw,ClawPivot clawPivot,ClawWrist clawWrist,Elevator elevator,RobotState scoreMode){
        addCommands(
            claw.setStateCommand(ClawStates.Intoke),
            Commands.parallel(
                clawPivot.changeStateCommand(RobotState.CORAL_ELEVATOR_AVOIDANCE),
                clawWrist.changeStateCommand(RobotState.CORAL_ELEVATOR_AVOIDANCE)
            ),
            new WaitUntilCommand( () -> (clawPivot.isAtPose() && clawWrist.isAtPose())),
            //Elevator to pose
            new WaitCommand(0.2),
            Commands.parallel(
                 
                clawPivot.changeStateCommand(scoreMode),
                clawWrist.changeStateCommand(scoreMode)
            ),
            new WaitUntilCommand( () -> (clawPivot.isAtPose() && clawWrist.isAtPose()))
        );
    }
}
