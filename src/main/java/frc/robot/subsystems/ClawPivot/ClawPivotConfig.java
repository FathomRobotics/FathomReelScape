package frc.robot.subsystems.ClawPivot;

import edu.wpi.first.math.controller.PIDController;

public class ClawPivotConfig {
    public static PIDController ClawPivotPID = new PIDController(0.001, 0, 0);
    public static int ClawPivotMotorID =  5;
}
