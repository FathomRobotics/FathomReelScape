package frc.robot.commands;

import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Claw.Claw;

//TODO fix this command 
public class IntakeCommand extends Command {
    private PS5Controller driver2;
    private boolean teleop;
    private Claw claw;

    public IntakeCommand(PS5Controller drvier2, Claw claw){
        this.driver2 = driver2;
        this.claw = claw;
    }

    public IntakeCommand(Claw claw){
        this.teleop = false;
        this.claw = claw;
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        super.execute();
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        if (!this.teleop){
            return claw.getLimitSwitchBroken();
        }
        return false;
    }
    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        if(this.teleop){
            if (this.driver2 != null){
                this.driver2.setRumble(RumbleType.kBothRumble, 01);
            }
        }
        super.end(interrupted);
    }
    
}
