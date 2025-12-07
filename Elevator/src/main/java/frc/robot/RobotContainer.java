// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ElevatorStateMachine;
import frc.robot.subsystems.ElevatorStateMachine.ElevatorState;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick; 
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  CommandXboxController cont = new CommandXboxController(1);
  private final Elevator elevator = new Elevator(cont);

  private final ElevatorStateMachine stateMachine = new ElevatorStateMachine(elevator);

  /** 
   * We create commands beforehand just for organization purposes so the calls to the commands
   * in the button bindings looks cleaner. We also use Commands.deferredProxy() so that the
   * tryState method isn't evaluated here, but rather when a button is pressed
   * Explanation by SuperNURDs here: https://www.chiefdelphi.com/t/frc-3255-supernurds-2025-build-thread/477499/95
  */
  Command MOVING_TO_INTAKE = Commands.deferredProxy(
      () -> stateMachine.tryState(ElevatorState.MOVING_TO_INTAKE));

  Command MOVING_TO_L2 = Commands.deferredProxy(
      () -> stateMachine.tryState(ElevatorState.MOVING_TO_L2));

  Command MOVING_TO_L3 = Commands.deferredProxy(
      () -> stateMachine.tryState(ElevatorState.MOVING_TO_L3));

  Command MOVING_TO_L4 = Commands.deferredProxy(
      () -> stateMachine.tryState(ElevatorState.MOVING_TO_L4));

  Command HOLDING = Commands.deferredProxy(
      () -> stateMachine.tryState(ElevatorState.HOLDING));

  Command MANUAL_HOLDING = Commands.deferredProxy(
      () -> stateMachine.tryState(ElevatorState.HOLDING));

  Command MANUAL = Commands.deferredProxy(
      () -> stateMachine.tryState(ElevatorState.MANUAL));

  Command IDLE = Commands.deferredProxy(
      () -> stateMachine.tryState(ElevatorState.IDLE));

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }


  private void configureBindings() {
    Trigger revLimitSwitchPressed = new Trigger(() -> elevator.revLimitSwitchPressed());

    //This logic should be fixed, hopefully it works
    BooleanSupplier setpointSupplier = () -> elevator.atSetpoint();
    DoubleSupplier leftJoySupplier = () -> cont.getLeftY();

    Trigger atSetpoint = new Trigger(setpointSupplier);
    Trigger manualControl = new Trigger(() -> elevator.isInRange(leftJoySupplier));

    revLimitSwitchPressed.onTrue(elevator.resetEncoder());
    revLimitSwitchPressed.onTrue(IDLE);

    cont.x().onTrue(MOVING_TO_INTAKE);
    cont.a().onTrue(MOVING_TO_L2);
    cont.b().onTrue(MOVING_TO_L3);
    cont.y().onTrue(MOVING_TO_L4);

    manualControl.onTrue(MANUAL).onFalse(MANUAL_HOLDING);
    // manualControl.and(() -> !elevator.revLimitSwitchPressed()).onFalse(MANUAL_HOLDING);

    //bring the elevator to a holding state if it is at setpoint and not in the intake state
    //i think this logic is right??

    // atSetpoint.and(() -> !stateMachine.intakeIdleState()).onTrue(new ParallelCommandGroup(HOLDING, elevator.rumbleCommand()));
    atSetpoint.and(() -> !stateMachine.intakeIdleState()).onTrue(HOLDING.alongWith(elevator.rumbleCommand()));

    //otherwise if the elevator was in the moving to intake state, bring the elevator
    //state to idle. 
    atSetpoint.and(() -> stateMachine.intakeIdleState()).onTrue(elevator.rumbleCommand());

    // atSetpoint.onTrue(elevator.rumbleCommand());


    //make the controller rumble when the elevator reaches a setpoint
    // atSetpoint.onTrue(elevator.rumbleCommand());

  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return elevator.setHeight(ElevatorConstants.L4_HEIGHT);
  }

  //TODO make homing command
  public void resetEncoder() {
    elevator.resetEncoderNotCommand();
  }
}
