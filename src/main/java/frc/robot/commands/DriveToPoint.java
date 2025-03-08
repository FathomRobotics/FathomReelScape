package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain.CommandSwerveDrivetrain;

public class DriveToPoint extends Command {
    private CommandSwerveDrivetrain drivetrain;
    private Pose2d wantedPose;
    public DriveToPoint(CommandSwerveDrivetrain drivetrain,Pose2d wantedPose){
        this.drivetrain = drivetrain;
        this.wantedPose = wantedPose;
        addRequirements(this.drivetrain);

    }

    @Override
    public void cancel() {
        // TODO Auto-generated method stub
        super.cancel();
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        
        super.end(interrupted);
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        super.execute();
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return super.isFinished();
    }
    
}
