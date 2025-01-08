package frc.robot.subsystems.Claw;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.WantedGamepiece;


public class Claw extends SubsystemBase{

    public ClawStates currentState = ClawStates.Intaking;
    public WantedGamepiece wantedGamepiece = WantedGamepiece.Coral;

    private SparkMax LeftClawNeo = new SparkMax(ClawConfig.LeftClawNeoId, MotorType.kBrushless);
    private SparkMax RightClawNeo = new SparkMax(ClawConfig.RightClawNeoId, MotorType.kBrushless);

    private SparkMaxConfig LeftClawConfig = new SparkMaxConfig();
    @SuppressWarnings("unused")
    private SparkMaxConfig rightClawConfig = new SparkMaxConfig();
    
    private DoubleSolenoid leftFrontSolenoid  = new DoubleSolenoid(3,PneumaticsModuleType.CTREPCM, ClawConfig.LeftFrontSolonoidForwardId,ClawConfig.LeftFrontSolonoidReverseId);
    private DoubleSolenoid rightFrontSolenoid = new DoubleSolenoid(3,PneumaticsModuleType.CTREPCM,ClawConfig.RightFrontSolonoidForwardId,ClawConfig.RightFrontSolonoidReverseId);

    private DoubleSolenoid leftBackSolenoid = new DoubleSolenoid(3,PneumaticsModuleType.CTREPCM,ClawConfig.LeftBackSolonoidForwardId,ClawConfig.LeftBackSolonoidBackwardsId);
    private DoubleSolenoid rightBackSolenoid = new DoubleSolenoid(3,PneumaticsModuleType.CTREPCM,ClawConfig.RightBackSolonoidForwardId,ClawConfig.RightBackSolonoidReverseId);

    private DigitalInput limitSwitch = new DigitalInput(ClawConfig.LimitSwitchId);
    
    public Claw(){
        this.LeftClawConfig.inverted(true);
        CoralIntake();
    }

    @Override
    public void periodic() {
        if(limitSwitch.get() && this.currentState != ClawStates.Outtake){
            this.currentState = ClawStates.Intoke;
        }else{
            switch (wantedGamepiece) {
                case Coral:
                    CoralIntake();
                    break;
                case Algae:
                    AlgaeIntake();
    
                default:
                    
                    break;
            }
        }
        
       
        switch (currentState) {
            case Intaking:
                this.LeftClawNeo.set(1);
                this.RightClawNeo.set(1);
                break;
            case Intoke:
                this.LeftClawNeo.set(0.3);
                this.RightClawNeo.set(0.3);
                break;

            case Outtake:
                this.LeftClawNeo.set(-1);
                this.RightClawNeo.set(-1);
                break;
            default:
                this.LeftClawNeo.set(0);
                this.RightClawNeo.set(0);
                break;
        }
        
        super.periodic();
    }
  
    public void CoralIntake(){
        this.leftFrontSolenoid.set(Value.kReverse);
        this.rightFrontSolenoid.set(Value.kReverse);

        this.leftBackSolenoid.set(Value.kForward);
        this.rightBackSolenoid.set(Value.kForward);
    }

    public void AlgaeIntake(){
        this.leftFrontSolenoid.set(Value.kForward);
        this.rightFrontSolenoid.set(Value.kForward);

        this.leftBackSolenoid.set(Value.kReverse);
        this.rightBackSolenoid.set(Value.kReverse);
    }

    public boolean getLimitSwitchBroken(){
        return this.limitSwitch.get();
    }
}
