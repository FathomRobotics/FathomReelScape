// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
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
    CameraServer.startAutomaticCapture();
    
  }

  @Override
  public void robotPeriodic() {
    this.m_robotContainer.wrist.setBrake(true);

    this.m_robotContainer.climb.setPower(m_robotContainer.operator.getRightY());
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
    
    var driveState = m_robotContainer.drivetrain.getState();
    double headingDeg = driveState.Pose.getRotation().getDegrees();
    double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);
    if(this.isDisabled()){
      LimelightHelpers.SetIMUMode("limelight-four",1);
      LimelightHelpers.SetRobotOrientation("limelight-four", headingDeg, 0, 0, 0, 0, 0);
    }else{
      LimelightHelpers.SetIMUMode("limelight-four",2);
    }
    LimelightHelpers.SetRobotOrientation("limelight-three", headingDeg,0,0,0,0,0);
    var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-four");
    
    var ll3Measurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-three");
   
    if (llMeasurement != null && llMeasurement.tagCount > 0 && Math.abs(omegaRps) < 1) {
     m_robotContainer.drivetrain.addVisionMeasurement(llMeasurement.pose, llMeasurement.timestampSeconds,VecBuilder.fill(1.1,1.1,9999999));
    }

    if (ll3Measurement != null && ll3Measurement.tagCount > 0 && Math.abs(omegaRps) < 1) {
      m_robotContainer.drivetrain.addVisionMeasurement(ll3Measurement.pose, ll3Measurement.timestampSeconds,VecBuilder.fill(1.1,1.1,9999999));
    }
    if(m_robotContainer.claw.getLimitSwitchBroken()){
      m_robotContainer.led.setLedMode(m_robotContainer.led.intoke);
    }else{
      m_robotContainer.led.setLedMode(m_robotContainer.led.passive);
    }
    //\if (RightTagCam.isReal()){
    //  EstimatedRobotPose RightPose = RhtTagCam.getEstimatedGlobalPose().get();
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
    SmartDashboard.putBoolean("Has piece", m_robotContainer.claw.getLimitSwitchBroken());
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
    LimelightHelpers.SetIMUMode("limelight-four",1);
    LimelightHelpers.SetRobotOrientation("limelight-four", m_robotContainer.drivetrain.getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
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

      this.m_robotContainer.elevator.goToPosition(RobotState.STOW.getElevatorPose());
      this.m_robotContainer.wrist.changeTargetPose(RobotState.STOW.getElevatorPose());
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
