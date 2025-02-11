package frc.robot.subsystems.ElevatorPivot;

public enum ElevatorPivotPoseRequest {
    
    StartPose(0),
    HPPickup(0),
    Score(0),
    GroundPickup(0);


    private int value;
     
    ElevatorPivotPoseRequest(int value){
        this.value = value;
    }

    public int getValue(){
        return this.value;
    }
}
