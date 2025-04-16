package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Led extends SubsystemBase {
    public Spark ledController = new Spark(7);
    public double passive = -0.41;
    public double intoke = -0.23;
    public Led(){
        setLedMode(-0.23);
    }

    public void setLedMode(double mode){
        this.ledController.set(mode);
    }

    public Command setModeCommand(double mode){
        return Commands.runOnce(
            () -> this.setLedMode(mode)

        );
    }


}
