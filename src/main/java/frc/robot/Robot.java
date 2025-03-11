// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Vision.Eyes;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  public Eyes LeftTagCam = new Eyes("LeftTagCam", Constants.LeftElevatorCamPose);
  public Eyes RightTagCam = new Eyes("RightTagCam", Constants.RightElevatorCamPose);

  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    if (LeftTagCam.isReal()){
      EstimatedRobotPose LeftPose = LeftTagCam.getEstimatedGlobalPose().get();

      if(LeftPose != null){
        m_robotContainer.drivetrain.addVisionMeasurement(LeftPose.estimatedPose.toPose2d(), LeftPose.timestampSeconds);
      }
    }else{
      System.out.println("Left camera isnt connected");
    }
    
    if (RightTagCam.isReal()){
      EstimatedRobotPose RightPose = RightTagCam.getEstimatedGlobalPose().get();
      if(RightPose != null){
        m_robotContainer.drivetrain.addVisionMeasurement(RightPose.estimatedPose.toPose2d(), RightPose.timestampSeconds);
      }
    }else{
      System.out.println("Right camera isnt connected");
    }
    

    
    
   
    CommandScheduler.getInstance().run();
    
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

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
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
