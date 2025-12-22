// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Robot;

public class Elevator extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  SparkMax sparkMax = new SparkMax(25, MotorType.kBrushless);

  RelativeEncoder elevatorEncoder;
  SparkClosedLoopController pid;

  double setpoint;
  int counter = 0;
  boolean teleopControl;

  SparkLimitSwitch forwardLimitSwitch;
  SparkLimitSwitch reverseLimitSwitch;

  double input;
  DoubleSupplier joy;
  CommandXboxController xboxController;

  public Elevator(CommandXboxController xboxController) {
    SparkMaxConfig config = new SparkMaxConfig();
    elevatorEncoder = sparkMax.getEncoder();
    pid = sparkMax.getClosedLoopController();
    this.xboxController = xboxController;

    LimitSwitchConfig limitSwitchConfig = new LimitSwitchConfig();
    ClosedLoopConfig closedLoopConfig = new ClosedLoopConfig();

    //TODO make working soft limits for the elevator
    SoftLimitConfig softLimitConfig = new SoftLimitConfig();

    softLimitConfig.reverseSoftLimit(0);
    softLimitConfig.forwardSoftLimit(26.7);

    softLimitConfig.forwardSoftLimitEnabled(true);
    softLimitConfig.reverseSoftLimitEnabled(true);

    forwardLimitSwitch = sparkMax.getForwardLimitSwitch();
    reverseLimitSwitch = sparkMax.getReverseLimitSwitch();

    closedLoopConfig.feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    //BAD CONSTANTS TUNE PLS
    closedLoopConfig.pidf(
      0.1, 
      0, 
      0.8, 
      0.00025);

    //TODO find values for max velocity and max acceleration
    // closedLoopConfig.maxMotion.maxVelocity(0).maxAcceleration(0);

    limitSwitchConfig.forwardLimitSwitchType(Type.kNormallyClosed);
    limitSwitchConfig.reverseLimitSwitchType(Type.kNormallyClosed);

    limitSwitchConfig.forwardLimitSwitchEnabled(true);
    limitSwitchConfig.reverseLimitSwitchEnabled(true);



    config.smartCurrentLimit(39);

    //We invert the encoder since normally, the encoder read negative values as the elevator
    //was lifted up
    config.inverted(true);

    config.apply(limitSwitchConfig);
    config.apply(closedLoopConfig);
    config.apply(softLimitConfig);



    sparkMax.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  public Command resetEncoder() {
    return this.runOnce(() -> {
      elevatorEncoder.setPosition(0);
    });
  }

  //TODO this probably will need to be tuned
  public boolean atSetpoint() {
    double pos = elevatorEncoder.getPosition()*ElevatorConstants.conversionFactor;

    return MathUtil.isNear(setpoint, pos, 0.75);
  }

  public boolean isInRange(DoubleSupplier dub) {
    return Math.abs(dub.getAsDouble()) > 0.05;
  }

  public boolean revLimitSwitchPressed() {
    return reverseLimitSwitch.isPressed();
  } 

  public boolean forwardLimitSwitchPressed() {
    return forwardLimitSwitch.isPressed();
  }
  //for a setpoint in inches
  public Command setHeight(double setpoint) {
    return this.runOnce(() -> {
      
      this.setpoint = MathUtil.clamp(setpoint, 0, 26.5);

      //we divide the setpoint by the conversion factor since 
      //the setpoint is in inches, and the conversion factor is inches/motor rotation
      //so inches/inches/motor rotation gives motor rotations
      pid.setReference(this.setpoint/ElevatorConstants.conversionFactor, ControlType.kPosition);

    });
  }
  public Command manualControl(DoubleSupplier joy) {

    
    return this.run(() -> {
      
      input = MathUtil.applyDeadband(joy.getAsDouble(), 0.1);

      if (setpoint < 2 || setpoint > 26) {
        setpoint += (input * -1) * 0.10;
      }
      else {
        setpoint += (input*-1)*0.25;
      }

      MathUtil.clamp(setpoint, 0, 26);
      

      pid.setReference(this.setpoint/ElevatorConstants.conversionFactor, ControlType.kPosition);

    });
  }


  //TODO idk if this works test later
  public Command rumbleCommand() {

    Runnable onInitialize = () -> {
      counter = 0;
    };

    Runnable onExecute = () -> {
      xboxController.setRumble(RumbleType.kBothRumble, .5);
      counter++;
    };

    Consumer<Boolean> onEnd = interrupted -> {
      xboxController.setRumble(RumbleType.kBothRumble, 0);
    };

    BooleanSupplier isFinished = () -> {
      return counter >= 25;
    };

    Command rumble = new FunctionalCommand(onInitialize, onExecute, onEnd, isFinished);

    return rumble;
  }

  public double setpoint() {
    return setpoint;
  }


  public Command turnOnMotor() {
    return this.runOnce(() -> {
      sparkMax.set(1);
  
    });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("relative encoder position", elevatorEncoder.getPosition()*ElevatorConstants.conversionFactor);
    SmartDashboard.putBoolean("Reverse Limit Switch Pressed", revLimitSwitchPressed());
    SmartDashboard.putBoolean("Forward Limit Switch Pressed", forwardLimitSwitchPressed());
    SmartDashboard.putNumber("setpoint", setpoint);
    SmartDashboard.putNumber("motor output", sparkMax.getAppliedOutput());
    SmartDashboard.putNumber("current robot time", Robot.currentTime);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

}
