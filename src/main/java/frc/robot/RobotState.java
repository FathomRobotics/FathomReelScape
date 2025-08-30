package frc.robot;

public enum RobotState {
    HUMAN_PLAYER_INTAKE_PREP(0,0),
    L1_PREP(-91,2),
    L2_PREP(-20,6), // (-15, 6)
    L3_PREP(-20,9.3), // (-15, 9.4)
    L4_PREP(-5,15),
    CORAL_ELEVATOR_AVOIDANCE(0,0),
    PREPED(0,0),
    SCORE_STATION_CORAL(0,0),
    SCORE_GROUND_CORAL(-95,14.9),
    SHOOT_CORAL(0,0),
    GROUND_INTAKE_ALGAE(-30,0),
    GROUND_INTAKE_CORAL(-6,0),
    L2_CLEAN_ALGAE(-30,6),
    L3_CLEAN_ALGAE(-30,9.4),
    SCORE_BARGE(-110,15.8),
    SCORE_PROCCESSOR(0,0),
    HUMAN_PLAYER_INTAKE(-110,4.5), // (-89, 4.5)
    STOW(-130,2); // (-120, 2)


    private double ElevatorPose;
    private double IntakeSpeed; 
    private double ClawWristPose;

    RobotState(double ClawWristPose, double ElevatorPose){
        this.ElevatorPose = ElevatorPose;
        this.ClawWristPose = ClawWristPose;
      
    }

    public double getElevatorPose(){
        return this.ElevatorPose;
    }

    public double getIntakeSpeed(){
        return this.IntakeSpeed;
    }
    public double getClawWristPose(){
        return this.ClawWristPose;
    }
}
