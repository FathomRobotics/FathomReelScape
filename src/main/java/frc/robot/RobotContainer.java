// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.Generated.TunerConstants;
import frc.robot.commands.DriveToPoint;
import frc.robot.subsystems.Drivetrain.CommandSwerveDrivetrain;

public class RobotContainer {


    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    //private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
   // private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandPS5Controller joystick = new CommandPS5Controller(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    PathPlannerPath aAlign;
    PathPlannerPath bAlign;
    PathPlannerPath cAlign;
    PathPlannerPath dAlign;
    PathPlannerPath lAlign;
    PathPlannerPath kAlign;


    /* Path follower */
    private final AutoFactory autoFactory;
    private final AutoRoutines autoRoutines;
    private final AutoChooser autoChooser = new AutoChooser();

    

    RobotConfig config;
    
    

  public RobotContainer() {
        try{
          config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
          // Handle exception as needed
          e.printStackTrace();
        }
        autoFactory = drivetrain.createAutoFactory();
        autoRoutines = new AutoRoutines(autoFactory);
        
        autoChooser.addRoutine("SimplePath Auto", autoRoutines::simplePathAuto);
        autoChooser.addRoutine("TopAuto", autoRoutines::ThreeCoralTop);



        SmartDashboard.putData("Auto Chooser", autoChooser);

        try {
          aAlign = PathPlannerPath.fromPathFile("aAlign");
          bAlign = PathPlannerPath.fromPathFile("bAlign");
          cAlign = PathPlannerPath.fromPathFile("cAlign");
          dAlign = PathPlannerPath.fromPathFile("dAlign");
          lAlign = PathPlannerPath.fromPathFile("lAlign");
          kAlign = PathPlannerPath.fromPathFile("kAlign");
          



        } catch (FileVersionException e) {
          // TODO Auto-generated catch block
          e.printStackTrace();
        } catch (IOException e) {
          // TODO Auto-generated catch block
          e.printStackTrace();
        } catch (ParseException e) {
          // TODO Auto-generated catch block
          e.printStackTrace();
        }

        configureBindings();
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(
             //Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        

        joystick.cross().whileTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        joystick.L3().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        
        joystick.povLeft().and(joystick.L1()).whileTrue(new DriveToPoint().cmd(kAlign));
        joystick.povLeft().and(joystick.R1()).whileTrue(new DriveToPoint().cmd(lAlign));

        joystick.povDown().and(joystick.L1()).whileTrue(new DriveToPoint().cmd(aAlign));
        joystick.povDown().and(joystick.R1()).whileTrue(new DriveToPoint().cmd(bAlign));

        joystick.povRight().and(joystick.L1()).whileTrue(new DriveToPoint().cmd(cAlign));
        joystick.povRight().and(joystick.R1()).whileTrue(new DriveToPoint().cmd(dAlign));



        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.povDown().and(joystick.cross()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.povDown().and(joystick.square()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.povUp().and(joystick.cross()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.povUp().and(joystick.square()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        

        drivetrain.registerTelemetry(logger::telemeterize);

        
  }



  public Command getAutonomousCommand() {
     return autoChooser.selectedCommand();
  }

  
}

