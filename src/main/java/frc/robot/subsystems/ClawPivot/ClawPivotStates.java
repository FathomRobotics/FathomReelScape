package frc.robot.subsystems.ClawPivot;

public enum ClawPivotStates {
    StartPose(0),
    GroundIntake(0),
    LOne(0),
    LTwo(0),
    LThree(0),
    LFour(0),
    HumanPlayer(0);

    private int value;
     
    ClawPivotStates(int value){
        this.value = value;
    }

    public int getValue(){
        return this.value;
    }

}
