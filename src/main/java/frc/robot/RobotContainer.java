// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.io.IOException;
import java.text.ParseException;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.Generated.TunerConstants;
import frc.robot.commands.DriveToPoint;
import frc.robot.subsystems.Led;
import frc.robot.subsystems.Claw.Claw;
import frc.robot.subsystems.Claw.ClawStates;
import frc.robot.subsystems.Claw.ClawTuah;
import frc.robot.subsystems.ClawWrist.ClawWrist;
import frc.robot.subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.StevesRevenge.Climb;

public class RobotContainer {


    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(2).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity was .75

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1 ) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    //private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
   // private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandPS5Controller joystick = new CommandPS5Controller(0);

    public final CommandPS5Controller operator = new CommandPS5Controller(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Elevator elevator = new Elevator();
    public final ClawTuah claw = new ClawTuah();
    public final ClawWrist wrist = new ClawWrist();
    public final Climb climb = new Climb();
    public final Led led = new Led();
    public RobotState lastScorePose = RobotState.L4_PREP;
    private boolean isCoral = true;

    public double elevatorOffset = 0;

    PathPlannerPath aAlign;
    PathPlannerPath bAlign;
    PathPlannerPath cAlign;
    PathPlannerPath dAlign;
    PathPlannerPath lAlign;
    PathPlannerPath kAlign;
    PathPlannerPath eAlign;
    PathPlannerPath fAlign;
    PathPlannerPath gAlign;
    PathPlannerPath hAlign;
    PathPlannerPath iAlign;
    PathPlannerPath jAlign;



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
        autoRoutines = new AutoRoutines(autoFactory,this.elevator,this.wrist,this.claw);
        
        autoChooser.addRoutine("TopAuto", autoRoutines::ThreeCoralTop);
        autoChooser.addRoutine("Single Middle Auto", autoRoutines::middleAuto);



        SmartDashboard.putData("Auto Chooser", autoChooser);

     try {
          aAlign = PathPlannerPath.fromPathFile("aAlign");
          bAlign = PathPlannerPath.fromPathFile("BAlign");
          cAlign = PathPlannerPath.fromPathFile("cAlign");
          dAlign = PathPlannerPath.fromPathFile("dAlign");
          lAlign = PathPlannerPath.fromPathFile("LAlign");
          kAlign = PathPlannerPath.fromPathFile("KAlign");
          
          eAlign = PathPlannerPath.fromPathFile("eAlign");
          fAlign = PathPlannerPath.fromPathFile("fAlign");
          gAlign = PathPlannerPath.fromPathFile("gAlign");
          hAlign = PathPlannerPath.fromPathFile("hAlign");
          iAlign = PathPlannerPath.fromPathFile("iAlign");
          jAlign = PathPlannerPath.fromPathFile("jAlign");
        

        } catch (FileVersionException e) {
          e.printStackTrace();
        } catch (IOException e) {
          
          e.printStackTrace();
        } catch (Exception e) {
          e.printStackTrace();
          }
        configureBindings();
  }

  private void configureBindings() {
    


    operator.povRight().onTrue(
     claw.setStateCommand(ClawStates.Intaking)
    );
    
    
    drivetrain.setDefaultCommand(
        //Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

      
        joystick.povUp().whileTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        joystick.L3().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        

        operator.L2().onTrue(
          new ConditionalCommand(
            Commands.sequence(
              this.claw.setStateCommand(ClawStates.Intaking),
              this.elevator.goToPositionCommand(RobotState.HUMAN_PLAYER_INTAKE.getElevatorPose() + getElevatorOffset()),
              this.wrist.goToPoseCommand(RobotState.HUMAN_PLAYER_INTAKE.getClawWristPose()),

              new WaitUntilCommand( () ->this.claw.getLimitSwitchBroken()),

              this.elevator.goToPositionCommand(RobotState.STOW.getElevatorPose() + getElevatorOffset()),
              this.wrist.goToPoseCommand(RobotState.STOW.getClawWristPose())
            )
          ,
            Commands.sequence(
              this.claw.setStateCommand(ClawStates.IntakeAlgae),
              this.elevator.goToPositionCommand(0)
              , 
              Commands.sequence(
                this.wrist.goToPoseCommand(RobotState.GROUND_INTAKE_ALGAE.getClawWristPose())
              )
            ),
             () -> isCoral
          )
          
        );

        operator.povDown().onTrue(
          Commands.sequence(
            this.wrist.goToPoseCommand(RobotState.STOW.getClawWristPose()),
              this.elevator.goToPositionCommand(RobotState.STOW.getElevatorPose() + getElevatorOffset()),
              this.claw.setStateCommand(ClawStates.Intoke)
          )
        );

        operator.povUp().onTrue(
          Commands.sequence(
            this.elevator.goToPositionCommand(0),
            this.wrist.goToPoseCommand(-10)
          )
        );

        operator.cross().onTrue(
          Commands.sequence(
            new ConditionalCommand(
              this.elevator.goToPositionCommand(RobotState.L2_PREP.getElevatorPose() + getElevatorOffset()), 
              Commands.sequence(
                this.elevator.goToPositionCommand(RobotState.L2_CLEAN_ALGAE.getElevatorPose()),
                this.claw.setStateCommand(ClawStates.IntakeAlgae)
              )
              
              , () -> isCoral),
              new InstantCommand(() -> this.lastScorePose = RobotState.L2_PREP),
              wrist.goToPoseCommand(RobotState.L2_PREP.getClawWristPose())
        ));

        operator.circle().onTrue(
          Commands.sequence(  
          new ConditionalCommand(
            this.elevator.goToPositionCommand(RobotState.L3_PREP.getElevatorPose() + getElevatorOffset()), 
            Commands.sequence(
              this.claw.setStateCommand(ClawStates.IntakeAlgae),
              this.elevator.goToPositionCommand(RobotState.L3_CLEAN_ALGAE.getElevatorPose())
            )
            , 
            () -> isCoral),
            new InstantCommand(() -> this.lastScorePose = RobotState.L3_PREP),
            wrist.goToPoseCommand(RobotState.L3_PREP.getClawWristPose())
        ));
       
        operator.triangle().onTrue(
          Commands.sequence(
          new ConditionalCommand(
            Commands.sequence(
            this.elevator.goToPositionCommand(RobotState.L4_PREP.getElevatorPose() + getElevatorOffset()),
            
            wrist.goToPoseCommand(RobotState.L4_PREP.getClawWristPose())), 
            Commands.sequence(
            this.elevator.goToPositionCommand(RobotState.SCORE_BARGE.getElevatorPose()),
            wrist.goToPoseCommand(RobotState.SCORE_BARGE.getClawWristPose())
            )
          , () -> isCoral),
          new InstantCommand(() -> this.lastScorePose = RobotState.L4_PREP)
          
        ));

        operator.square().onTrue(
          new InstantCommand(
            () -> changeIntakeType(!isCoral)
          )
        );


         
        operator.R2().whileTrue(
            this.claw.setStateCommand(ClawStates.Outtake)
        );

      
        operator.L3().whileTrue(
          Commands.sequence(
            new InstantCommand( () -> elevator.enableOverride( () -> -operator.getLeftY()))
          )
        );
        
      
        


       
        

        //auto align

        //left Align vals: tx: +0.57° ty: +0.42° ta: +2.915%
        //right Align vals: tx: +9.69° ty: +1.04° ta: 4



      joystick.povLeft().and(joystick.L1()).whileTrue(new DriveToPoint().cmd(kAlign));
      joystick.povLeft().and(joystick.R1()).whileTrue(new DriveToPoint().cmd(lAlign));

      joystick.povDown().and(joystick.L1()).whileTrue(new DriveToPoint().cmd(aAlign));
      joystick.povDown().and(joystick.R1()).whileTrue(new DriveToPoint().cmd(bAlign));

      joystick.povRight().and(joystick.L1()).whileTrue(new DriveToPoint().cmd(cAlign));
      joystick.povRight().and(joystick.R1()).whileTrue(new DriveToPoint().cmd(dAlign));

      
      joystick.square().and(joystick.L1()).whileTrue(new DriveToPoint().cmd(jAlign));
      joystick.square().and(joystick.R1()).whileTrue(new DriveToPoint().cmd(iAlign));

      joystick.triangle().and(joystick.L1()).whileTrue(new DriveToPoint().cmd(hAlign));
      joystick.triangle().and(joystick.R1()).whileTrue(new DriveToPoint().cmd(gAlign));

      joystick.circle().and(joystick.L1()).whileTrue(new DriveToPoint().cmd(fAlign));
      joystick.circle().and(joystick.R1()).whileTrue(new DriveToPoint().cmd(eAlign));
        //driver 2 stuff
        
    
        
       drivetrain.registerTelemetry(logger::telemeterize);

        
  }



  public Command getAutonomousCommand() {
     return autoChooser.selectedCommand();
  }

  private void changeIntakeType(boolean isCoral){
    this.isCoral = isCoral;
  }

  public double getElevatorOffset(){
    return this.elevatorOffset;
  }
  
  public void changeOffset(double amount){
    this.elevatorOffset += amount;
  }
}

