// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {

  NetworkTableInstance inst = NetworkTableInstance.getDefault();

  private Command m_autonomousCommand;

  //public Eyes LeftTagCam = new Eyes("LeftCamera", Constants.LeftElevatorCamPose);
  //public Eyes RightTagCam = new Eyes("RightTagCam", Constants.RightElevatorCamPose);

  private final RobotContainer m_robotContainer;


  public Robot() {
    m_robotContainer = new RobotContainer();
    
  }

  @Override
  public void robotPeriodic() {
    this.m_robotContainer.wrist.setBrake(true);
    /* 
    if (LeftTagCam.isReal()){
      if(LeftTagCam.getEstimatedGlobalPose().isPresent()){
      Pose3d LeftPose = LeftTagCam.getEstimatedGlobalPose().get().estimatedPose;
      double timestamp = LeftTagCam.getEstimatedGlobalPose().get().timestampSeconds;
      System.out.println(LeftPose);

      if(LeftPose != null){
        m_robotContainer.drivetrain.addVisionMeasurement(LeftPose.toPose2d(), timestamp);
      }
    }
    }else{
     // System.out.println("Left camera isnt connected");
    }
*/
    this.m_robotContainer.elevator.periodic();
    this.m_robotContainer.claw.periodic();
    this.m_robotContainer.wrist.periodic();
    
    //if (RightTagCam.isReal()){
    //  EstimatedRobotPose RightPose = RightTagCam.getEstimatedGlobalPose().get();
   ////   if(RightPose != null){
   //     m_robotContainer.drivetrain.addVisionMeasurement(RightPose.estimatedPose.toPose2d(), RightPose.timestampSeconds);
     // }
   // }else{
   //   System.out.println("Right camera isnt connected");
   // }
    

    
    //m_robotContainer.elevator.VoltageTest(m_robotContainer.operator.getLeftY() * 12);
  

    SmartDashboard.putNumber("Voltage Given To ELevator", m_robotContainer.operator.getLeftY() * 12);
    SmartDashboard.putNumber("Elevator Pose", m_robotContainer.elevator.getPosition());
    SmartDashboard.putNumber("Wrist pose",m_robotContainer.wrist.getPose());
    //SmartDashboard.putBoolean("Has piece", m_robotContainer.claw.getLimitSwitchBroken());
   // if(m_robotContainer.operator.circle().getAsBoolean()){
    //  SmartDashboard.putNumber("kv", m_robotContainer.elevator.kvTest() / 10);
   // }else{
   //   m_robotContainer.elevator.VoltageTest(0);
   // }
    CommandScheduler.getInstance().run();
    
    
    
  }

  @Override
  public void disabledInit() {
    this.m_robotContainer.wrist.setBrake(false);
  }

  @Override
  public void disabledPeriodic() {

  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

      this.m_robotContainer.elevator.goToPosition(1);
      this.m_robotContainer.wrist.changeTargetPose(0);
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
    //add the poses here and put them on smart dashboard


  }

  @Override
  public void testExit() {}
}
