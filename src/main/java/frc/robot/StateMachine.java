package frc.robot;

public class StateMachine {
    public static RobotState currentRobotState;
    private RobotState lastRobotState = RobotState.STOW;

    public StateMachine(){
        currentRobotState = RobotState.STOW;
    }

    public void update(RobotState newState){
        currentRobotState = newState;
    }

    public boolean checkForNewState(){
        return currentRobotState != lastRobotState;
    }
}
