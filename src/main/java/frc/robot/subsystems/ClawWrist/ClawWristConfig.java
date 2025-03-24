package frc.robot.subsystems.ClawWrist;

import edu.wpi.first.math.controller.PIDController;

public class ClawWristConfig {
    public static int ClawWristMotorID = 16;
    public static PIDController clawWristController = new PIDController(0.001, 0, 0);

}
