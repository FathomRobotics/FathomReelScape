package frc.robot;

public enum RobotState {
    HUMAN_PLAYER_INTAKE_PREP(0,0),
    L1_PREP(-24,1),
    L2_PREP(-19,7),
    L3_PREP(-19,10), // was 19
    L4_PREP(-19,14),
    CORAL_ELEVATOR_AVOIDANCE(0,0),
    PREPED(0,0),
    SCORE_STATION_CORAL(0,0),
    SCORE_GROUND_CORAL(0,0),
    SHOOT_CORAL(0,0),
    GROUND_INTAKE_ALGAE(0,0),
    GROUND_INTAKE_CORAL(-29,0),
    L2_CLEAN_ALGAE(0,0),
    L3_CLEAN_ALGAE(0,0),
    SCORE_BARGE(0,0),
    SCORE_PROCCESSOR(0,0),
    HUMAN_PLAYER_INTAKE(-80,4.5),
    STOW(-90,2);


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
