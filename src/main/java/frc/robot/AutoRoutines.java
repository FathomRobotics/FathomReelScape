package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Claw.Claw;
import frc.robot.subsystems.Claw.ClawStates;
import frc.robot.subsystems.Claw.ClawTuah;
import frc.robot.subsystems.ClawWrist.ClawWrist;
import frc.robot.subsystems.Elevator.Elevator;

public class AutoRoutines {
    private final AutoFactory m_factory;
    private Elevator elevator;
    private ClawWrist wrist;
    private ClawTuah claw;
    public AutoRoutines(AutoFactory factory,Elevator elevator, ClawWrist wrist, ClawTuah claw) {
        m_factory = factory;
        this.elevator = elevator;
        this.wrist = wrist;
        this.claw = claw;
    }

   
    public AutoRoutine ThreeCoralTop(){

        final AutoRoutine routine = m_factory.newRoutine("TopAuto");
        final AutoTrajectory preload = routine.trajectory("TopMiddleStart");
        final AutoTrajectory getFromstation = routine.trajectory("JToTop");
        final AutoTrajectory scoreAtReefL = routine.trajectory("TopToL");
        final AutoTrajectory LToStation = routine.trajectory("LToTop");
        final AutoTrajectory KToStation = routine.trajectory("KToTop");
        final AutoTrajectory scoreAtReefK = routine.trajectory("TopToK");

        routine.active().onTrue(
            Commands.sequence(

                preload.resetOdometry(),
                Commands.parallel(
                preload.cmd(),
                Commands.sequence(
                    new WaitCommand(0.7),
                    elevator.goToPositionCommand(RobotState.L4_PREP.getElevatorPose()),
                    wrist.goToPoseCommand(RobotState.L4_PREP.getClawWristPose())
                )
                )
            )
        );

        preload.done().onTrue(
            Commands.sequence(
                claw.setStateCommand(ClawStates.Outtake),
                new WaitCommand(0.4),
                wrist.goToPoseCommand(RobotState.STOW.getClawWristPose()),
                new WaitCommand(0.5),
                elevator.goToPositionCommand(RobotState.STOW.getElevatorPose()),
                new WaitCommand(1),
                new ParallelCommandGroup(
                    getFromstation.cmd(),
                    elevator.goToPositionCommand(RobotState.HUMAN_PLAYER_INTAKE.getElevatorPose()),
                    wrist.goToPoseCommand(RobotState.HUMAN_PLAYER_INTAKE.getClawWristPose()),
                    claw.setStateCommand(ClawStates.Intaking)
                )
                
                 

            )
            );
 
        getFromstation.done().onTrue(
            Commands.sequence(
                new WaitUntilCommand(() -> claw.getLimitSwitchBroken()),
                elevator.goToPositionCommand(RobotState.STOW.getElevatorPose()),
                wrist.goToPoseCommand(RobotState.STOW.getClawWristPose()),
                new ParallelCommandGroup(
                    scoreAtReefL.cmd(),
                    Commands.sequence(
                        new WaitCommand(0.55),
                        elevator.goToPositionCommand(RobotState.L4_PREP.getElevatorPose()),
                        wrist.goToPoseCommand(RobotState.L4_PREP.getClawWristPose())
                    )
                )
               

            )
        );
         
        scoreAtReefL.done().onTrue(
            Commands.sequence(
                claw.setStateCommand(ClawStates.Outtake),
                new WaitCommand(0.5),
                Commands.parallel(
                LToStation.cmd(),
                Commands.sequence(
                    new WaitCommand(0.2),
                    elevator.goToPositionCommand(RobotState.HUMAN_PLAYER_INTAKE.getElevatorPose()),
                    wrist.goToPoseCommand(RobotState.HUMAN_PLAYER_INTAKE.getClawWristPose()),
                    claw.setStateCommand(ClawStates.Intaking)
                )
                )
                
            )
        );
        
        LToStation.done().onTrue(
            Commands.sequence(
                new WaitUntilCommand(() -> claw.getLimitSwitchBroken()),
                elevator.goToPositionCommand(RobotState.STOW.getElevatorPose()),
                wrist.goToPoseCommand(RobotState.STOW.getClawWristPose()),
                new ParallelCommandGroup(
                    scoreAtReefK.cmd(),
                    Commands.sequence(
                        new WaitCommand(0.55),
                        elevator.goToPositionCommand(RobotState.L4_PREP.getElevatorPose()),
                        wrist.goToPoseCommand(RobotState.L4_PREP.getClawWristPose())
                    )
                )
            )
        );

        scoreAtReefK.done().onTrue(
            Commands.sequence(
                claw.setStateCommand(ClawStates.Outtake)
            )
        );
        


        return routine;

    }

    
    public AutoRoutine middleAuto(){
        final AutoRoutine routine = m_factory.newRoutine("Middle Auto");
        final AutoTrajectory goToReef = routine.trajectory("PreloadMiddleG");

        routine.active().onTrue(
            Commands.sequence(

                goToReef.resetOdometry(),
                Commands.parallel(
                goToReef.cmd(),
                elevator.goToPositionCommand(RobotState.L4_PREP.getElevatorPose()),
                wrist.goToPoseCommand(RobotState.L4_PREP.getClawWristPose())
                )
            )
        );

        goToReef.done().onTrue(
            Commands.sequence(
             claw.setStateCommand(ClawStates.Outtake),
             new WaitCommand(1),
             wrist.goToPoseCommand(RobotState.L4_PREP.getClawWristPose()),
             new WaitCommand(1),
             elevator.goToPositionCommand(RobotState.L2_PREP.getElevatorPose())
            )
        );



        return routine;
    }
 
}