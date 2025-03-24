package frc.robot;

public enum RobotState {
    HUMAN_PLAYER_INTAKE_PREP(0,0,0),
    L1_PREP(0,0,0),
    L2_PREP(0,0,0),
    L3_PREP(30,-10,5.3),
    L4_PREP(0,0,0),
    CORAL_ELEVATOR_AVOIDANCE(0,0,0),
    PREPED(0,0,0),
    SCORE_STATION_CORAL(0,0,0),
    SCORE_GROUND_CORAL(0,0,0),
    SHOOT_CORAL(0,0,0),
    GROUND_INTAKE_ALGAE(0,0,0),
    GROUND_INTAKE_CORAL(50,-35,0),
    L2_CLEAN_ALGAE(0,0,0),
    L3_CLEAN_ALGAE(0,0,0),
    SCORE_BARGE(0,0,0),
    SCORE_PROCCESSOR(0,0,0),
    HUMAN_PLAYER_INTAKE(0,0,0),
    STOW(90,35,1);

    private double ClawPivotPose;
    private double ElevatorPose;
    private double IntakeSpeed; 
    private double ClawWristPose;

    RobotState(double ClawPivotPose, double ClawWristPose, double ElevatorPose){
        this.ClawPivotPose = ClawPivotPose;
        this.ElevatorPose = ElevatorPose;
        this.ClawWristPose = ClawWristPose;
      
    }

    public double getClawPose(){
        return this.ClawPivotPose;
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
