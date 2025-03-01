package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Claw.Claw;
import frc.robot.subsystems.ClawPivot.ClawPivot;
import frc.robot.subsystems.Elevator.Elevator;

public class AutoRoutines {
    private final AutoFactory m_factory;

    public AutoRoutines(AutoFactory factory) {
        m_factory = factory;
    }

    public AutoRoutine simplePathAuto() {
        final AutoRoutine routine = m_factory.newRoutine("SimplePath Auto");
        final AutoTrajectory preload = routine.trajectory("PreloadTopJ");
        final AutoTrajectory getFromstation = routine.trajectory("JToTop");
        final AutoTrajectory scoreFromStation = routine.trajectory("TopToA");

        routine.active().onTrue(
            Commands.sequence(

                preload.resetOdometry(),
                preload.cmd()
            )
        );

        preload.done().onTrue(
            Commands.sequence(
                new WaitCommand(1),
                getFromstation.cmd()
            )
            );

        getFromstation.done().onTrue(
            Commands.sequence(
                new WaitCommand(1),
                scoreFromStation.cmd()
            )
            
           );


        return routine;
    }

    public AutoRoutine testThreeCoral(ClawPivot clawPivot, Claw claw, Elevator elevator){

        final AutoRoutine routine = m_factory.newRoutine("SimplePath Auto");
        final AutoTrajectory preload = routine.trajectory("PreloadTopJ");
        final AutoTrajectory getFromstation = routine.trajectory("JToTop");
        final AutoTrajectory scoreAtReefL = routine.trajectory("TopToL");
        final AutoTrajectory KToStation = routine.trajectory("KToTop");
        final AutoTrajectory scoreAtReefK = routine.trajectory("TopToK");

        
        


        return routine;

    }
 
}