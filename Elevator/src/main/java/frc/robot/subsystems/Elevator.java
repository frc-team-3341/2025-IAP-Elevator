// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  SparkMax sparkMax = new SparkMax(25, MotorType.kBrushless);
  RelativeEncoder elevatorEncoder;
  SparkClosedLoopController pid;

  double conversionFactor = 5.5/45;

  public Elevator() {
    SparkMaxConfig config = new SparkMaxConfig();
    elevatorEncoder = sparkMax.getEncoder();
    pid = sparkMax.getClosedLoopController();

    LimitSwitchConfig limitSwitchConfig = new LimitSwitchConfig();
    ClosedLoopConfig closedLoopConfig = new ClosedLoopConfig();

    closedLoopConfig.feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    //TODO tune ts lool
    closedLoopConfig.pidf(
      0, 
      0, 
      0, 
      0);

    //TODO find values for max velocity and max acceleration
    closedLoopConfig.maxMotion.maxVelocity(0).maxAcceleration(0);

    limitSwitchConfig.forwardLimitSwitchType(Type.kNormallyClosed);
    limitSwitchConfig.reverseLimitSwitchType(Type.kNormallyClosed);

    limitSwitchConfig.forwardLimitSwitchEnabled(true);
    limitSwitchConfig.reverseLimitSwitchEnabled(true);

    config.apply(limitSwitchConfig);
    config.apply(closedLoopConfig);

    sparkMax.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  //for a setpoint in inches
  public Command setHeight(double setpoint) {
    return this.runOnce(() -> {

      pid.setReference(setpoint*conversionFactor, SparkBase.ControlType.kMAXMotionPositionControl);

    });
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("relative encoder position", elevatorEncoder.getPosition()*conversionFactor);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
