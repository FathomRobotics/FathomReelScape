package frc.robot.subsystems.Elevator;


import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Elevator extends SubsystemBase {
 


   private TalonFX motorA = new TalonFX(ElevatorConfig.MotorAcanID, "rio");
   private TalonFX motorB = new TalonFX(ElevatorConfig.MotorbCanID, "rio");

  private double MaxExtensionValue = 15.8;
  private double RotationsPerIn = MaxExtensionValue / 84; //84 in max extension


  private double targetPose = 0;
  private final MotionMagicVoltage motionControl = new MotionMagicVoltage(0);
  private final Follower follow = new Follower(ElevatorConfig.MotorAcanID, false);

  private DoubleSupplier ds;

  private boolean override = false;

  public Elevator(){
       TalonFXConfiguration cfg = new TalonFXConfiguration();

        motorA.setNeutralMode(NeutralModeValue.Brake);
        motorB.setNeutralMode(NeutralModeValue.Brake);

      cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        
       var limitConfigs = new CurrentLimitsConfigs();


       limitConfigs.StatorCurrentLimit = 90;
       limitConfigs.StatorCurrentLimitEnable = true;


      /* Configure gear ratio */
      FeedbackConfigs fdb = cfg.Feedback;
      fdb.SensorToMechanismRatio = 9.02; // 12.8 rotor rotations per mechanism rotation 9.02 to 1 


      /* Configure Motion Magic */

      var motionMagicConfigs = cfg.MotionMagic;
      motionMagicConfigs.MotionMagicCruiseVelocity =  10; // Target cruise velocity of 80 rps
      motionMagicConfigs.MotionMagicAcceleration =  10 * 2 ; // Target acceleration of 160 rps/s (0.5 seconds)
      motionMagicConfigs.MotionMagicJerk = 10 * 2* 3; // Target jerk of 1600 rps/s/s (0.1 seconds)
     


      Slot0Configs slot0 = cfg.Slot0;
      slot0.kS = 0.4; // Add 0.25 V output to overcome static friction was 0.28
      slot0.kV = 4; // A velocity target of 1 rps results in 0.12 V output was 8.48
      slot0.kA = 0.02; // An acceleration of 1 rps/s requires 0.01 V output
      slot0.kP = 25; // A position error of 0.2 rotations results in 12 V output
      slot0.kI = 0; // No output for integrated error
      slot0.kD = 0; // A velocity error of 1 rps results in 0.5 V output was 0.5
      


      StatusCode status = StatusCode.StatusCodeNotInitialized;
      StatusCode status2 = StatusCode.StatusCodeNotInitialized; 
      for (int i = 0; i < 5; ++i) {
        status = motorA.getConfigurator().apply(cfg);
        status2 = motorB.getConfigurator().apply(cfg); 
        motorA.getConfigurator().apply(limitConfigs);
        motorB.getConfigurator().apply(limitConfigs);
        if (status.isOK() && status2.isOK()) break;
      }
      if (!status.isOK()) {
        System.out.println("Could not configure device. Error: " + status.toString());
      }
 
      motorA.setPosition(0);
      motorB.setPosition(0);
    }


    public void goToPosition (double newPosition){
      this.override = false;
      this.targetPose =  newPosition;

    }

    public boolean getAtPose(){
      return Math.abs(this.motorA.getPosition().getValueAsDouble() - this.targetPose) < 0.05;
    }
    public double getPostition() {
      return motorA.getPosition().getValueAsDouble();
    }


    public Command goToPositionCommand(double target){
      return Commands.runOnce( ()-> goToPosition(target));
    }

    public void VoltageTest(double power){
        this.override = true;
        motorA.setVoltage(power * 5);
        motorB.setControl(follow);
    }

    public void enableOverride(DoubleSupplier power){
      this.ds = power;
      this.override = true;
    }

    public double kvTest(){
      motorA.setVoltage(10);
      motorB.setControl(follow);

      return motorA.getVelocity().getValueAsDouble();
    }

    public double getPosition(){
        return this.motorA.getPosition().getValueAsDouble();
    }


    @Override
    public void periodic(){
      if(!this.override){
        motorA.setControl(motionControl.withPosition(targetPose).withSlot(0));
        motorB.setControl(follow);
  
      }else{
        this.VoltageTest(this.ds.getAsDouble());
      }

      
    }





  }
