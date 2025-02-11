package frc.robot.subsystems.ElevatorPivot;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LastKnownPositions;

public class ElevatorPivot extends SubsystemBase {
    private TalonFX pivotMotorA = new TalonFX(ElevatorPivotConfig.pivotMotorA);
    private TalonFX pivotMotorB = new TalonFX(ElevatorPivotConfig.pivotMotorB);

    private final VoltageOut m_sysIdControl = new VoltageOut(0);
    
    private final PositionVoltage positionControl = new PositionVoltage(LastKnownPositions.ElevatorPivotLastKnownPose).withSlot(1);
   


    private TalonFXConfiguration cfg = new TalonFXConfiguration();

    
   

    public ElevatorPivot(){
        FeedbackConfigs fdb = cfg.Feedback;
        fdb.SensorToMechanismRatio = ElevatorPivotConfig.motorToDiamter;

        
        cfg.Slot0.kP = 0.001; // An error of 1 rotation results in 2.4 V output
        cfg.Slot0.kI = 0; // No output for integrated error
        cfg.Slot0.kD = 0;
         StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = pivotMotorA.getConfigurator().apply(cfg);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply configs, error code: " + status.toString());
        }
    
            
    }

    public void setPoseRequest(double pose){
        LastKnownPositions.ElevatorPivotLastKnownPose = pose;
        pivotMotorA.setControl(this.positionControl.withPosition(pose));
        pivotMotorB.setControl(new Follower(ElevatorPivotConfig.pivotMotorA, false));
    }

    public boolean atRequestedPose(){
        return Math.abs(this.pivotMotorA.getPosition().getValueAsDouble() - LastKnownPositions.ElevatorPivotLastKnownPose) < 0.05; 

    }

    public Command ElevatorPivotPoseCommand(double pose){
        return runOnce(
           () -> setPoseRequest(pose)
        );
    }
    public void powerMotors(Voltage volts){
        pivotMotorA.setControl(m_sysIdControl.withOutput(volts));
        pivotMotorB.setControl(new Follower(ElevatorPivotConfig.pivotMotorA, false));

    }
}
