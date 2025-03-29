package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Claw.Claw;
import frc.robot.subsystems.Claw.ClawStates;
import frc.robot.subsystems.ClawWrist.ClawWrist;
import frc.robot.subsystems.Elevator.Elevator;

public class AutoRoutines {
    private final AutoFactory m_factory;
    private Elevator elevator;
    private ClawWrist wrist;
    private Claw claw;
    public AutoRoutines(AutoFactory factory,Elevator elevator, ClawWrist wrist, Claw claw) {
        m_factory = factory;
        this.elevator = elevator;
        this.wrist = wrist;
        this.claw = claw;
    }

    public AutoRoutine simplePathAuto() {
        final AutoRoutine routine = m_factory.newRoutine("SimplePath Auto");
        final AutoTrajectory preload = routine.trajectory("PreloadMiddleG");

        routine.active().onTrue(
            Commands.sequence(

                preload.resetOdometry(),
                preload.cmd()
            )
        );

        preload.done().onTrue(
            Commands.sequence(
                new WaitCommand(1)
            )
            );

        


        return routine;
    }

    public AutoRoutine ThreeCoralTop(){

        final AutoRoutine routine = m_factory.newRoutine("TopAuto");
        final AutoTrajectory preload = routine.trajectory("PreloadTopJ");
        final AutoTrajectory getFromstation = routine.trajectory("JToTop");
        final AutoTrajectory scoreAtReefL = routine.trajectory("TopToL");
        final AutoTrajectory LToStation = routine.trajectory("LToTop");
        final AutoTrajectory KToStation = routine.trajectory("KToTop");
        final AutoTrajectory scoreAtReefK = routine.trajectory("TopToK");

        routine.active().onTrue(
            Commands.sequence(

                preload.resetOdometry(),
                preload.cmd()
            )
        );
        
        preload.done().onTrue(
            Commands.sequence(
                getFromstation.cmd()
            )
        );

        getFromstation.done().onTrue(
            Commands.sequence(
                scoreAtReefL.cmd()
            )
        );

        scoreAtReefL.done().onTrue(
            Commands.sequence(
                LToStation.cmd()
            )
        );

        LToStation.done().onTrue(
            Commands.sequence(
                scoreAtReefK.cmd()
            )
        );

        scoreAtReefK.done().onTrue(
            Commands.sequence(
                KToStation.cmd()
            )
        );
        


        return routine;

    }
    public AutoRoutine ThreeCoralBottom(){
        final AutoRoutine routine = m_factory.newRoutine("BottomAuto");
        //final AutoTrajectory preload = routine.trajectory("PreloadBottomF");
        //final AutoTrajectory FToBottom = routine.trajectory("FToBottom");
        //final AutoTrajectory BottomToC = routine.trajectory("BottomToC");
        //final AutoTrajectory CToBottom = routine.trajectory("CToBottom");
        //final AutoTrajectory BottomToD = routine.trajectory("BottomToD");
        //final AutoTrajectory DToBottom = routine.trajectory("DToBottom");

        



        return routine;
    }

    public AutoRoutine GroundIntakeTopAuto(){
        final AutoRoutine routine = m_factory.newRoutine("Ground Intake top auto");

        final AutoTrajectory preload = routine.trajectory("PreloadTopJ");
        final AutoTrajectory TopCoralPickup = routine.trajectory("TopGroundCoral");
        final AutoTrajectory TopCoralScore = routine.trajectory("TopGroundCoralPlace");
        final AutoTrajectory MiddleCoral = routine.trajectory("MiddleGroundCoral");

        routine.active().onTrue(
            Commands.sequence(

                preload.resetOdometry(),
                Commands.parallel(
                preload.cmd(),
                elevator.goToPositionCommand(RobotState.L3_PREP.getElevatorPose()),
                wrist.goToPoseCommand(RobotState.L3_PREP.getClawWristPose())
                )
            )
        );

        preload.done().onTrue(
            Commands.sequence(
                new WaitCommand(0.5),
                claw.setStateCommand(ClawStates.Outtake),
                new WaitCommand(0.8),
                wrist.goToPoseCommand(RobotState.STOW.getClawWristPose()),
                new WaitCommand(0.5),
                elevator.goToPositionCommand(RobotState.STOW.getElevatorPose()),
                new WaitCommand(2)
               // TopCoralPickup.cmd()

            )
            );
    
        //TopCoralPickup.done().onTrue(
          //  Commands.sequence(
          //      TopCoralScore.cmd()
          //  )
      //  );

        /*TopCoralScore.done().onTrue(
            Commands.sequence(
                MiddleCoral.cmd()
            )
        );
        */

        return routine;
    }
 
}