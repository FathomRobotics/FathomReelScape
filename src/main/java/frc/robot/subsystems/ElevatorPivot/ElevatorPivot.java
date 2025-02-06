package frc.robot.subsystems.ElevatorPivot;

import static edu.wpi.first.units.Units.Volt;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class ElevatorPivot extends SubsystemBase {
    private TalonFX pivotMotorA = new TalonFX(ElevatorPivotConfig.pivotMotorA);
    private TalonFX pivotMotorB = new TalonFX(ElevatorPivotConfig.pivotMotorB);
    private final VoltageOut m_sysIdControl = new VoltageOut(0);

    private TalonFXConfiguration cfg = new TalonFXConfiguration();

    private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);

    private final SysIdRoutine m_sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,         // Use default ramp rate (1 V/s)
                Volt.of(4), // Reduce dynamic voltage to 4 to prevent brownout
                null,          // Use default timeout (10 s)
                                       // Log state with Phoenix SignalLogger class
                state -> SignalLogger.writeString("state", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                volts -> powerMotors(volts),
                null,
                this
            )
        );

    public ElevatorPivot(){
        FeedbackConfigs fdb = cfg.Feedback;
        fdb.SensorToMechanismRatio = ElevatorPivotConfig.motorToDiamter;
        MotionMagicConfigs mm  = cfg.MotionMagic;
            
    }

    public Command elevatorPivotSysIdQuasistatic(SysIdRoutine.Direction direction){
        return m_sysIdRoutine.quasistatic(direction);
    }
    public Command elevatorPivotSysIdDynamic(SysIdRoutine.Direction direction){
        return m_sysIdRoutine.dynamic(direction);
    }

    public void powerMotors(Voltage volts){
        pivotMotorA.setControl(m_sysIdControl.withOutput(volts));
        pivotMotorB.setControl(new Follower(ElevatorPivotConfig.pivotMotorA, false));

    }
}
