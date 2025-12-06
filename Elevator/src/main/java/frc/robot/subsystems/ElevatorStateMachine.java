package frc.robot.subsystems;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorStateMachine extends SubsystemBase {
    public static ElevatorStates currentElevatorState;
    Elevator elevator;

    ElevatorStateMachine stateMachine = this;

    public ElevatorStateMachine(Elevator elevator) {
        this.elevator = elevator;
        currentElevatorState = ElevatorStates.IDLE;




    }

    public void setElevatorState(ElevatorStates newState) {
        currentElevatorState = newState;
    }

    public ElevatorStates getElevatorState(){
        return currentElevatorState;
    }

    public Command tryState(ElevatorStates newState){
        switch (newState) {
            case MOVING_TO_INTAKE:
                return elevator.setHeight(Constants.ElevatorConstants.INTAKE_HEIGHT);
            case MOVING_TO_L2:
                return elevator.setHeight(Constants.ElevatorConstants.L2_HEIGHT);
            case MOVING_TO_L3:  
                return elevator.setHeight(Constants.ElevatorConstants.L3_HEIGHT);
            case MOVING_TO_L4:
                return elevator.setHeight(Constants.ElevatorConstants.L4_HEIGHT);
            case HOLDING:
                
            case MANUAL:
                
            case IDLE:

            default:
                return elevator.setHeight(0);
                


                
        }
    }


        
} 