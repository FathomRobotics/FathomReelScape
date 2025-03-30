package frc.robot.subsystems.StevesRevenge;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Climb extends SubsystemBase {
    private SparkMax ClimbMotor = new SparkMax(ClimbConfig.ClimbId,MotorType.kBrushless);
    
    private SparkClosedLoopController closedLoopController;
    private SparkMaxConfig climbMotorConfig;

    public Climb(){
        closedLoopController = ClimbMotor.getClosedLoopController();
        climbMotorConfig = new SparkMaxConfig();
        
        climbMotorConfig.smartCurrentLimit(80);
        climbMotorConfig.idleMode(IdleMode.kBrake);
        
        

        climbMotorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .outputRange(-1, 1)
           
            .p(0.05)
            .i(0)
            .d(0);

        ClimbMotor.configure(climbMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    }

    public void setPower(double power){
        this.ClimbMotor.set(power);
    }




}
