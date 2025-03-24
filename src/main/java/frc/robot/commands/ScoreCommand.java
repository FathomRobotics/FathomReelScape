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

public class ScoreCommand extends SequentialCommandGroup{
    public ScoreCommand(Claw claw, ClawPivot pivot, ClawWrist wrist, Elevator elevator,RobotState scoreMode){
        if(scoreMode != RobotState.SCORE_STATION_CORAL){
            if (scoreMode == RobotState.SHOOT_CORAL){
                addCommands(
                    Commands.sequence(
                        claw.setStateCommand(ClawStates.Shooting),
                        new WaitUntilCommand( ()-> !claw.getLimitSwitchBroken()),
                        new WaitCommand(0.3)
                    )
                );
            }else{
                addCommands(
                    Commands.sequence(
                        //elevator to pose
                        Commands.parallel(
                            pivot.goToPoseCommand(40),
                            wrist.goToPoseCommand(0)
                        ),
                        new WaitCommand(0.4),
                        claw.setStateCommand(ClawStates.Outtake),
                        Commands.parallel(
                            pivot.goToPoseCommand(RobotState.STOW.getClawPose()),
                            wrist.goToPoseCommand(20)
                        ),
                        new WaitCommand(0.8),
                        Commands.parallel(
                            pivot.goToPoseCommand(RobotState.STOW.getClawPose()),
                            wrist.goToPoseCommand(RobotState.STOW.getClawWristPose())
                        )
                        
                    )
                        
                );
            

            }


        }else{

            addCommands(
                    Commands.sequence(
                        elevator.goToPositionCommand(scoreMode.getElevatorPose()),
                        new WaitUntilCommand( () ->elevator.getAtPose()),
                        claw.setStateCommand(ClawStates.Outtake),
                        new WaitUntilCommand( ()-> !claw.getLimitSwitchBroken()),
                        new WaitCommand(0.3),
                        elevator.goToPositionCommand(RobotState.STOW.getElevatorPose())
                        
                    )
                );
        }
    }
}
