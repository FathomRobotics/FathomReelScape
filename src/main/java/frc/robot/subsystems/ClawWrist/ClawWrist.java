package frc.robot.subsystems.ClawWrist;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;

public class ClawWrist extends SubsystemBase {
    private SparkMax pivotMotor = new SparkMax(ClawWristConfig.ClawWristMotorID,MotorType.kBrushless);
    private SparkClosedLoopController closedLoopController;
    private SparkMaxConfig pivotMotorConfig;
    private double targetPose = RobotState.STOW.getClawWristPose();

    private RobotState currentState = RobotState.STOW;
    public ClawWrist(){
        closedLoopController = pivotMotor.getClosedLoopController();
        pivotMotorConfig = new SparkMaxConfig();
        
        pivotMotorConfig.smartCurrentLimit(40);
        pivotMotorConfig.idleMode(IdleMode.kCoast);
        
        

        pivotMotorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .outputRange(-1, 1)
           
            .p(0.05)
            .i(0)
            .d(0);

        pivotMotor.configure(pivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        this.pivotMotor.getEncoder().setPosition(0);
    }

    public void updatePivotPose(double pose){
        this.closedLoopController.setReference(pose, ControlType.kPosition,ClosedLoopSlot.kSlot0);
    }

    public void changeState(RobotState newStates){
        this.currentState = newStates;

    }

    public void setRawPower(double power){
        this.pivotMotor.set(power);
    }

    public Command changeStateCommand(RobotState newState){
        return runOnce( () -> changeState(newState));
    }

    public Command goToPoseCommand(double pose){
        return runOnce( ()-> changeTargetPose(pose));
    }

    public void changeTargetPose(double pose){
        this.targetPose = pose;
    }
    public boolean isAtPose(){ 
        return Math.abs(this.pivotMotor.getAbsoluteEncoder().getPosition() - this.currentState.getClawWristPose()) < 0.05;
    }
    public double getPose(){
        return this.pivotMotor.get();
    }

    public void setBrake(boolean enable){
        if(enable){
           this.pivotMotorConfig.idleMode(IdleMode.kBrake);
        }else{
            this.pivotMotorConfig.idleMode(IdleMode.kCoast);
        }
       
    }
    @Override
    public void periodic() {
       updatePivotPose(this.targetPose);
    
        super.periodic();
    }
    
}
