// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ElevatorStateMachine;
import frc.robot.subsystems.ElevatorStateMachine.ElevatorState;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick; 
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ElevatorConstants;


import java.util.Set;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Elevator elevator = new Elevator();
  CommandJoystick joy = new CommandJoystick(0);
  CommandXboxController cont = new CommandXboxController(1);

  private final ElevatorStateMachine stateMachine = new ElevatorStateMachine(elevator);

  Command MOVING_TO_INTAKE = Commands.deferredProxy(
      () -> stateMachine.tryState(ElevatorState.MOVING_TO_INTAKE));
  Command MOVING_TO_L2 = Commands.deferredProxy(
      () -> stateMachine.tryState(ElevatorState.MOVING_TO_L2));
  Command MOVING_TO_L3 = Commands.deferredProxy(
      () -> stateMachine.tryState(ElevatorState.MOVING_TO_L3));
  Command MOVING_TO_L4 = Commands.deferredProxy(
      () -> stateMachine.tryState(ElevatorState.MOVING_TO_L4));



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }


  private void configureBindings() {
    Trigger revLimitSwitchPressed = new Trigger(() -> elevator.revLimitSwitchPressed());

    revLimitSwitchPressed.onTrue(elevator.resetEncoder());

    joy.button(3).onTrue(MOVING_TO_INTAKE);
    joy.button(1).onTrue(MOVING_TO_L2);
    joy.button(2).onTrue(MOVING_TO_L3);
    joy.button(4).onTrue(MOVING_TO_L4);

  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return elevator.setHeight(ElevatorConstants.L4_HEIGHT);
  }

  public void resetEncoder() {
    elevator.resetEncoderNotCommand();
  }
}
