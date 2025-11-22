// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static frc.robot.Constants.ElevatorConstants;


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


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }


  private void configureBindings() {
    Trigger revLimitSwitchPressed = new Trigger(() -> elevator.revLimitSwitchPressed());

    revLimitSwitchPressed.onTrue(elevator.resetEncoder());

    joy.button(3).onTrue(elevator.setHeight(ElevatorConstants.INTAKE_HEIGHT));
    joy.button(1).onTrue(elevator.setHeight(ElevatorConstants.L2_HEIGHT));
    joy.button(2).onTrue(elevator.setHeight(ElevatorConstants.L3_HEIGHT));
    joy.button(4).onTrue(elevator.setHeight(ElevatorConstants.L4_HEIGHT));
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return elevator.setHeight(ElevatorConstants.L4_HEIGHT);
  }

  public void resetEncoder() {
    elevator.resetEncoderNotCommand();
  }
}
