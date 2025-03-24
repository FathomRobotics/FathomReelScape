package frc.robot.subsystems.Claw;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.WantedGamepiece;


public class Claw extends SubsystemBase{

    public ClawStates currentState = ClawStates.Intoke;
    private WantedGamepiece wantedGamepiece = WantedGamepiece.Coral;

    private SparkMax RightClawNeo = new SparkMax(ClawConfig.RightClawNeoId, MotorType.kBrushless);
    private SparkMax LeftClawNeo = new SparkMax(ClawConfig.LeftClawNeoId, MotorType.kBrushless);

    private SparkMaxConfig LeftClawConfig = new SparkMaxConfig();
    
   
    private PIDController velocityController = new PIDController(0.1, 0, 0);

    private DigitalInput limitSwitch = new DigitalInput(ClawConfig.LimitSwitchId);
    
    public Claw(){
        
    
    }

    @Override
    public void periodic() {
        if(getLimitSwitchBroken() && this.currentState != ClawStates.Outtake && this.currentState != ClawStates.IntakeAlgae ){
            this.currentState = ClawStates.Intoke;
        }
        
    
        switch (currentState) {
            case Intaking:
                this.setIntakeVelocity(0.5);
                break;
            case Intoke:
                this.setIntakeVelocity(0);
                break;
            case Outtake:
                this.setIntakeVelocity(-0.5);
                break;
            case Shooting:
                this.setIntakeVelocity(0);
            case IntakeAlgae:
                this.setIntakeVelocity(0.3);
            default:
                this.setIntakeVelocity(0);
                break;
        }
        
        super.periodic();
    }



    public void setIntakeVelocity(double Value){
        

        this.RightClawNeo.set(Value);
        this.LeftClawNeo.set(-Value);
        
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
    public Command setClawPiece(WantedGamepiece pGamepiece){
        return runOnce( () -> setWantedGamePiece(pGamepiece));
    }

    public void setWantedGamePiece(WantedGamepiece pGamepiece){
        this.wantedGamepiece = pGamepiece;
    }
}
