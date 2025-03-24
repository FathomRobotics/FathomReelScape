// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.config.RobotConfig;
import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.Generated.TunerConstants;
import frc.robot.commands.ScoreCommand;
import frc.robot.subsystems.Claw.Claw;
import frc.robot.subsystems.Claw.ClawStates;
import frc.robot.subsystems.ClawPivot.ClawPivot;
import frc.robot.subsystems.ClawWrist.ClawWrist;
import frc.robot.subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator.Elevator;

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

    public final CommandPS5Controller operator = new CommandPS5Controller(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Elevator elevator = new Elevator();
    public final Claw claw = new Claw();
    public final ClawWrist wrist = new ClawWrist();
    public final ClawPivot pivot = new ClawPivot();

    public RobotState lastScorePose = RobotState.L4_PREP;
    private boolean isCoral = true;

    //PathPlannerPath aAlign;
    //PathPlannerPath bAlign;
    //PathPlannerPath cAlign;
    //PathPlannerPath dAlign;
    //PathPlannerPath lAlign;
    //PathPlannerPath kAlign;


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
        autoChooser.addRoutine("TopGroundAuto", autoRoutines::GroundIntakeTopAuto);



        SmartDashboard.putData("Auto Chooser", autoChooser);

    /*   try {
      //    aAlign = PathPlannerPath.fromPathFile("aAlign");
      //    bAlign = PathPlannerPath.fromPathFile("bAlign");
       //   cAlign = PathPlannerPath.fromPathFile("cAlign");
       //   dAlign = PathPlannerPath.fromPathFile("dAlign");
       //   lAlign = PathPlannerPath.fromPathFile("lAlign");
       //   kAlign = PathPlannerPath.fromPathFile("kAlign");
          



        } catch (FileVersionException e) {
          e.printStackTrace();
        } catch (IOException e) {
          
          e.printStackTrace();
        } catch (ParseException e) {
          e.printStackTrace();
        }

        */
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

      
        joystick.povUp().whileTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        joystick.L3().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        operator.square().onTrue(new InstantCommand(() -> changeIntakeType(!isCoral)));

        //operator.circle().onTrue(Commands.sequence(
        //  this.wrist.goToPoseCommand(0),
        //  this.pivot.goToPoseCommand(70)
        //  //this.elevator.goToPositionCommand(13)
      //  ));

      //  operator.triangle().onTrue(Commands.sequence(
      //    this.wrist.goToPoseCommand(20),
       //   this.pivot.goToPoseCommand(50)
          //this.elevator.goToPositionCommand(0)
      //  ));
        
        operator.L2().onTrue(
          new ConditionalCommand(
            Commands.sequence(
              this.claw.setStateCommand(ClawStates.Intaking),
              this.elevator.goToPositionCommand(1)
              ,
              Commands.sequence(
                this.wrist.goToPoseCommand(-35),
                this.pivot.goToPoseCommand(55)
              ),
              new WaitCommand(0.4),
              this.elevator.goToPositionCommand(0),
                new WaitCommand(0.8),
                this.elevator.goToPositionCommand(RobotState.STOW.getElevatorPose()),
                Commands.sequence(
                this.wrist.goToPoseCommand(35),
                this.pivot.goToPoseCommand(80)
             )
          ),
            Commands.sequence(
              this.claw.setStateCommand(ClawStates.IntakeAlgae),
              this.elevator.goToPositionCommand(0)
              ,
              Commands.sequence(
                this.wrist.goToPoseCommand(10),
                this.pivot.goToPoseCommand(40)
              )
            ),

             () -> isCoral

          )
          
        );

        operator.povDown().onTrue(
          Commands.sequence(
            this.wrist.goToPoseCommand(35),
              this.pivot.goToPoseCommand(90),
              this.elevator.goToPositionCommand(RobotState.STOW.getElevatorPose())
          )
        );

        /* 
        operator.cross().onTrue(
          Commands.sequence(
            new ConditionalCommand(
              this.elevator.goToPositionCommand(RobotState.L2_PREP.getElevatorPose()), 
              this.elevator.goToPositionCommand(RobotState.L2_CLEAN_ALGAE.getElevatorPose())
              , () -> isCoral),
              new InstantCommand(() -> this.lastScorePose = RobotState.L2_PREP)
        ));
*/
        operator.circle().onTrue(
          Commands.sequence(
            this.wrist.goToPoseCommand(35),
            this.pivot.goToPoseCommand(90),
          new ConditionalCommand(
            this.elevator.goToPositionCommand(RobotState.L3_PREP.getElevatorPose()), 
            this.elevator.goToPositionCommand(RobotState.L3_CLEAN_ALGAE.getElevatorPose()), 
            () -> isCoral),
            new InstantCommand(() -> this.lastScorePose = RobotState.L3_PREP)
        ));
        /* 
        operator.triangle().onTrue(
          Commands.sequence(
          new ConditionalCommand(
            this.elevator.goToPositionCommand(RobotState.L4_PREP.getElevatorPose()), 
            this.elevator.goToPositionCommand(RobotState.SCORE_BARGE.getElevatorPose())
          , () -> isCoral),
          new InstantCommand(() -> this.lastScorePose = RobotState.L4_PREP)
        ));

        operator.povDown().onTrue(
          this.elevator.goToPositionCommand(RobotState.STOW.getElevatorPose())
        );  
 */
        operator.R2().onTrue(
          Commands.sequence(
            new ScoreCommand(claw,pivot,wrist,elevator,this.lastScorePose)
          )
        );

       
        

        //auto align
       // joystick.povLeft().and(joystick.L1()).whileTrue(new DriveToPoint().cmd(kAlign));
       // joystick.povLeft().and(joystick.R1()).whileTrue(new DriveToPoint().cmd(lAlign));

       // joystick.povDown().and(joystick.L1()).whileTrue(new DriveToPoint().cmd(aAlign));
        //joystick.povDown().and(joystick.R1()).whileTrue(new DriveToPoint().cmd(bAlign));

        //joystick.povRight().and(joystick.L1()).whileTrue(new DriveToPoint().cmd(cAlign));
        //joystick.povRight().and(joystick.R1()).whileTrue(new DriveToPoint().cmd(dAlign));

        
        //driver 2 stuff
        

        
        drivetrain.registerTelemetry(logger::telemeterize);

        
  }



  public Command getAutonomousCommand() {
     return autoChooser.selectedCommand();
  }

  private void changeIntakeType(boolean isCoral){
    this.isCoral = isCoral;
  }

  
}

