package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorStateMachine extends SubsystemBase {

    ElevatorState currentElevatorState;

    Elevator elevator;

    public ElevatorStateMachine(Elevator elevator) {

        this.elevator = elevator;
        currentElevatorState = ElevatorState.IDLE;

    }

    public void setElevatorState(ElevatorState newState) {
        currentElevatorState = newState;
    }

    public ElevatorState getElevatorState(){
        return currentElevatorState;
    }

    public boolean intakeIdleState() {
        return currentElevatorState.equals(ElevatorState.MOVING_TO_INTAKE) || 
        currentElevatorState.equals(ElevatorState.IDLE);
    }

    //Inspired by FRC Team 3255 SuperNURDs state machine implementation
    //build thread found here! https://www.chiefdelphi.com/t/frc-3255-supernurds-2025-build-thread/477499/95
    public Command tryState(ElevatorState newState){
        switch (newState) {
            case MOVING_TO_INTAKE:

                switch(currentElevatorState) {

                    case MOVING_TO_INTAKE:
                        return Commands.print("Already moving to intake!");
                    case IDLE:
                        return Commands.print("Already at intake position!");

                    case MOVING_TO_L2:
                    case MOVING_TO_L3:
                    case MOVING_TO_L4:
                    case HOLDING:
                    case MANUAL:
                        setElevatorState(ElevatorState.MOVING_TO_INTAKE);
                        return elevator.setHeight(ElevatorConstants.INTAKE_HEIGHT);
                }
                break;

            case MOVING_TO_L2:
                if (currentElevatorState == ElevatorState.MOVING_TO_L2) {
                    return Commands.print("Already moving to L2!");
                }
                
                else {
                    setElevatorState(ElevatorState.MOVING_TO_L2);
                    return elevator.setHeight(ElevatorConstants.L2_HEIGHT);
                }

            case MOVING_TO_L3:  
                if (currentElevatorState == ElevatorState.MOVING_TO_L3) {
                    return Commands.print("Already moving to L3!");
                }
            
                else {
                    setElevatorState(ElevatorState.MOVING_TO_L3);
                    return elevator.setHeight(ElevatorConstants.L3_HEIGHT);
                }

            case MOVING_TO_L4:
                if (currentElevatorState == ElevatorState.MOVING_TO_L4) {
                    return Commands.print("Already moving to L4!");
                }
            
                else {
                    setElevatorState(ElevatorState.MOVING_TO_L4);
                    return elevator.setHeight(ElevatorConstants.L4_HEIGHT);
                }

            case HOLDING: 
                setElevatorState(ElevatorState.HOLDING);
                return elevator.setHeight(elevator.setpoint());
                
            //TODO manual control for elevator
            case MANUAL:
                
            case IDLE:
                setElevatorState(ElevatorState.IDLE);

                //return an empty command for idle state
                return new InstantCommand(() -> {});            
        }
        
        return Commands.print("the state provided was invalid ruh roh (this shouldn't happen)");
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Current Elevator State", this.currentElevatorState.toString());
        SmartDashboard.putBoolean("intake state", intakeIdleState());
    }


    public enum ElevatorState{
        IDLE,

        //moving states
        MOVING_TO_INTAKE,
        MOVING_TO_L2,
        MOVING_TO_L3,
        MOVING_TO_L4,

        HOLDING,
        MANUAL
      
    }  
} 