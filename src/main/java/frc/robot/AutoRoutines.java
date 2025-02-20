package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoRoutines {
    private final AutoFactory m_factory;

    public AutoRoutines(AutoFactory factory) {
        m_factory = factory;
    }

    public AutoRoutine simplePathAuto() {
        final AutoRoutine routine = m_factory.newRoutine("SimplePath Auto");
        final AutoTrajectory preload = routine.trajectory("PreLoadTopJ");
        final AutoTrajectory getFromstation = routine.trajectory("JToTop");
        final AutoTrajectory scoreFromStation = routine.trajectory("TopToA");
        routine.active().onTrue(
            preload.resetOdometry()
                .andThen(preload.cmd()).andThen(
                    getFromstation.cmd()
                ).andThen(
                    scoreFromStation.cmd()
                )
        );
        return routine;
    }

    public Command simpleAutoCommandTest(){
        final AutoRoutine routine = m_factory.newRoutine("SimplePath Auto");
        final AutoTrajectory preload = routine.trajectory("PreLoadTopJ");
        
        routine.active().onTrue(
            preload.resetOdometry()
        );
        return new SequentialCommandGroup(
            preload.cmd()
        );
    }
}