package frc.robot.subsystems.Elevator;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private final XboxController m_joystick = new XboxController(0);
  private final TalonFX m_fx = new TalonFX(1, "canivore");

   private TalonFX motorA = new TalonFX(ElevatorConfig.MotorAcanID, "rio");

   private final MotionMagicVoltage m_mmReq = new MotionMagicVoltage(0);

  public Elevator(){
       TalonFXConfiguration cfg = new TalonFXConfiguration();

      /* Configure gear ratio */
      FeedbackConfigs fdb = cfg.Feedback;
      fdb.SensorToMechanismRatio = 12.8; // 12.8 rotor rotations per mechanism rotation

      /* Configure Motion Magic */
      MotionMagicConfigs mm = cfg.MotionMagic;
      mm.withMotionMagicCruiseVelocity(RotationsPerSecond.of(5)) // 5 (mechanism) rotations per second cruise
        .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(10)) // Take approximately 0.5 seconds to reach max vel
        // Take approximately 0.1 seconds to reach max accel 
        .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(100));

      Slot0Configs slot0 = cfg.Slot0;
      slot0.kS = 0.25; // Add 0.25 V output to overcome static friction
      slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
      slot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
      slot0.kP = 60; // A position error of 0.2 rotations results in 12 V output
      slot0.kI = 0; // No output for integrated error
      slot0.kD = 0.5; // A velocity error of 1 rps results in 0.5 V output

      StatusCode status = StatusCode.StatusCodeNotInitialized;
      for (int i = 0; i < 5; ++i) {
        status = motorA.getConfigurator().apply(cfg);
        if (status.isOK()) break;
      }
      if (!status.isOK()) {
        System.out.println("Could not configure device. Error: " + status.toString());
      }
    }

    public void teleopPeriodic() {
      double leftY = m_joystick.getLeftY();
    if (Math.abs(leftY) < 0.1) leftY = 0;

    m_fx.setControl(m_mmReq.withPosition(leftY).withSlot(0));
      if (m_joystick.getBButton()) {
        m_fx.setPosition(Rotations.of(1));
      }
    }

  }


