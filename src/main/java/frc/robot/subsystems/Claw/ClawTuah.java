package frc.robot.subsystems.Claw;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawTuah extends SubsystemBase {
    private TalonFX LeftIntake = new TalonFX(ClawConfig.LeftClawNeoId, "rio");
    private TalonFX RightIntake = new TalonFX(ClawConfig.RightClawNeoId,"rio");
    public ClawStates currentState = ClawStates.Intoke;

    private DigitalInput limitSwitch = new DigitalInput(ClawConfig.LimitSwitchId);

    public ClawTuah(){
        
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        this.LeftIntake.setNeutralMode(NeutralModeValue.Brake);
        this.RightIntake.setNeutralMode(NeutralModeValue.Brake);
        var limitConfigs = new CurrentLimitsConfigs();

        limitConfigs.StatorCurrentLimit = 90;
        limitConfigs.StatorCurrentLimitEnable = true;


        StatusCode status = StatusCode.StatusCodeNotInitialized;

        for (int i = 0; i < 5; ++i) {
            status = LeftIntake.getConfigurator().apply(cfg);
            RightIntake.getConfigurator().apply(cfg);
            LeftIntake.getConfigurator().apply(limitConfigs);
            RightIntake.getConfigurator().apply(limitConfigs);
            if (status.isOK()) break;
          }
          if (!status.isOK()) {
            System.out.println("Could not configure device. Error: " + status.toString());
          }
    }


    @Override
    public void periodic() {
        if(getLimitSwitchBroken() && this.currentState != ClawStates.Outtake){

            if (currentState == ClawStates.IntakeAlgae){

            }else{
                this.currentState = ClawStates.Intoke;
            }
            
        }
        
    
        switch (currentState) {
            case Intaking:
                this.setIntakeVelocity(0.2);
                break;
            case Intoke:
                this.setIntakeVelocity(0);
                break;
            case Outtake:
                this.setIntakeVelocity(-0.5); //was -0.2
                break;
            case Shooting:
                this.setIntakeVelocity(0);
                break;
            case IntakeAlgae:
                this.setIntakeVelocity(0.2);
                break;
            default:
                this.setIntakeVelocity(0);
                break;
        }
        
        super.periodic();
    }

    public boolean getLimitSwitchBroken(){
        return !this.limitSwitch.get();
     }


     public void setState(ClawStates newState){
       this.currentState = newState;
       
    }

    public Command setStateCommand(ClawStates newState){
        return runOnce( () -> setState(newState));
    }

    public void setIntakeVelocity(double Value){
        this.LeftIntake.setVoltage(Value * 12);
        this.RightIntake.setControl(new Follower(ClawConfig.LeftClawNeoId, true));
    }

}
