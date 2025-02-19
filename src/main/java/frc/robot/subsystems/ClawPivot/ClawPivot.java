package frc.robot.subsystems.ClawPivot;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawPivot extends SubsystemBase {
    private SparkMax pivotMotor = new SparkMax(ClawPivotConfig.ClawPivotMotorID,MotorType.kBrushless);
    private SparkClosedLoopController closedLoopController;
    private SparkMaxConfig pivotMotorConfig;

    private ClawPivotStates currentState = ClawPivotStates.StartPose;

    public ClawPivot(){
        closedLoopController = pivotMotor.getClosedLoopController();
        pivotMotorConfig = new SparkMaxConfig();
        
        pivotMotorConfig.smartCurrentLimit(40);
        
        pivotMotorConfig.encoder
        .positionConversionFactor(2 * Math.PI)
        .velocityConversionFactor(1);

        pivotMotorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .outputRange(-1, 1)
           
            .p(0.0001)
            .i(0)
            .d(0);

        pivotMotor.configure(pivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void updatePivotPose(double pose){
        this.closedLoopController.setReference(pose, ControlType.kMAXMotionPositionControl,ClosedLoopSlot.kSlot0);
    }

    public void changeState(ClawPivotStates newStates){
        this.currentState = newStates;

    }

    public void setRawPower(double power){
        this.pivotMotor.set(power);
    }

    public Command changeStateCommand(ClawPivotStates newState){
        return runOnce( () -> changeState(newState));
    }
    public boolean isAtPose(){ 
        return Math.abs(this.pivotMotor.getAbsoluteEncoder().getPosition() - this.currentState.getValue()) < 0.05;
    }

    @Override
    public void periodic() {
        updatePivotPose(this.currentState.getValue());
    
        super.periodic();
    }
    
}
