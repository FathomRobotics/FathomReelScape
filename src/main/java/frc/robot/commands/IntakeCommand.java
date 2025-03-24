package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotState;
import frc.robot.subsystems.Claw.Claw;
import frc.robot.subsystems.Claw.ClawStates;
import frc.robot.subsystems.ClawPivot.ClawPivot;
import frc.robot.subsystems.ClawWrist.ClawWrist;
import frc.robot.subsystems.Elevator.Elevator;

public class IntakeCommand extends SequentialCommandGroup {
    public IntakeCommand(Claw claw,ClawPivot clawPivot,ClawWrist clawWrist,Elevator elevator,RobotState scoreMode){
        addCommands(
            claw.setStateCommand(ClawStates.Intaking),
            Commands.parallel(
                clawPivot.changeStateCommand(RobotState.STOW),
                clawWrist.changeStateCommand(RobotState.STOW)
            ),
            new WaitUntilCommand( () -> (clawPivot.isAtPose() && clawWrist.isAtPose())),
            //add elevator down to intake pose
            claw.setStateCommand(ClawStates.Intaking),
            Commands.parallel(
                clawPivot.changeStateCommand(scoreMode),
                clawWrist.changeStateCommand(scoreMode)
            ),
            new WaitUntilCommand(() -> claw.getLimitSwitchBroken()),
            Commands.parallel(
                clawPivot.changeStateCommand(RobotState.STOW),
                clawWrist.changeStateCommand(RobotState.STOW)
            ),
            //add elevator to stow
            new WaitUntilCommand( () -> (clawPivot.isAtPose() && clawWrist.isAtPose()))
        );
    }

    
}
