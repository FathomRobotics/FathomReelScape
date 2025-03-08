package frc.robot.subsystems.Elevator;

public enum ElevatorStates {
    StartPose(0),
    GroundIntake(0),
    Stow(0),
    LOne(0),
    LTwo(0),
    LThree(0),
    LFour(0),
    HumanPlayer(0);

    private int value;
     
    ElevatorStates(int value){
        this.value = value;
    }

    public int getValue(){
        return this.value;
    }

}
