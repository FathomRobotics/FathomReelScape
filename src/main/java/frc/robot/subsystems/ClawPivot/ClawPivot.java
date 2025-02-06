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

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawPivot extends SubsystemBase {
    private SparkMax pivotMotor = new SparkMax(ClawPivotConfig.ClawPivotMotorID,MotorType.kBrushless);
    private SparkClosedLoopController closedLoopController;
    private SparkMaxConfig pivotMotorConfig;

    private ClawPivotStates currentState = ClawPivotStates.StartPose;

    public ClawPivot(){
        closedLoopController = pivotMotor.getClosedLoopController();
        pivotMotorConfig = new SparkMaxConfig();

        pivotMotorConfig.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);

    /*
     * Configure the closed loop controller. We want to make sure we set the
     * feedback sensor as the primary encoder.
     */
        pivotMotorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            // Set PID values for position control. We don't need to pass a closed loop
            // slot, as it will default to slot 0.
            .p(0.1)
            .i(0)
            .d(0)
            .outputRange(-1, 1)
            // Set PID values for velocity control in slot 1
            .p(0.0001, ClosedLoopSlot.kSlot1)
            .i(0, ClosedLoopSlot.kSlot1)
            .d(0, ClosedLoopSlot.kSlot1)
            .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
            .outputRange(-1, 1, ClosedLoopSlot.kSlot1);
        pivotMotor.configure(pivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void updatePivotPose(double pose){
        this.closedLoopController.setReference(pose, ControlType.kMAXMotionPositionControl,ClosedLoopSlot.kSlot0);
    }

    public void changeState(ClawPivotStates newStates){
        this.currentState = newStates;
    }

    @Override
    public void periodic() {
        updatePivotPose(this.currentState.getValue());
        
        super.periodic();
    }
    
}
