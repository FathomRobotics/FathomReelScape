package frc.robot;

public enum RobotState {
    HUMAN_PLAYER_INTAKE_PREP(0,0),
    L1_PREP(-91,1),
    L2_PREP(20,7.1),
    L3_PREP(15,10.7), // was 19
    L4_PREP(20,14),
    CORAL_ELEVATOR_AVOIDANCE(0,0),
    PREPED(0,0),
    SCORE_STATION_CORAL(0,0),
    SCORE_GROUND_CORAL(0,0),
    SHOOT_CORAL(0,0),
    GROUND_INTAKE_ALGAE(-17,0),
    GROUND_INTAKE_CORAL(-6,0),
    L2_CLEAN_ALGAE(0,0),
    L3_CLEAN_ALGAE(0,0),
    SCORE_BARGE(0,0),
    SCORE_PROCCESSOR(0,0),
    HUMAN_PLAYER_INTAKE(-84,4.4),
    STOW(-132,2);


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
