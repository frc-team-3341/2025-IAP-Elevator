// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.SwerveModuleIOSparkMax;
import frc.robot.RobotType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int PWM_CHANNEL = 4;
  }

  public static class ElevatorConstants {
    public static final double conversionFactor = 5.5/45;


    //Elevator setpoint heights in inches
    public static final double INTAKE_HEIGHT = 0;
    public static final double L2_HEIGHT = 9;
    public static final double L3_HEIGHT = 12.6;
    public static final double L4_HEIGHT = 26.5;
  }
  
  public static final RobotType ROBOT_TYPE = RobotType.ROBOT_2025_COMPETITION;

  public static final class SwerveModuleIOConfig{
        // Drive can ids start at front left from 1 and are odd
        // Turn can ids start at front left from 2 and are even
        // CANCoder ids start at front left from 10 and are sequential
        static SwerveModuleIOSparkMax moduleFL = new SwerveModuleIOSparkMax(//front left
                0, ROBOT_TYPE.ids[0][0], ROBOT_TYPE.ids[0][1], ROBOT_TYPE.ids[0][2], ROBOT_TYPE.moduleAngleOffsets[0], ROBOT_TYPE.moduleInverts[0]);
                //num // driveID // turnID // turnCANCoderID // turnEncoderOffset // invert
        static SwerveModuleIOSparkMax moduleFR = new SwerveModuleIOSparkMax(//front right
                1, ROBOT_TYPE.ids[1][0], ROBOT_TYPE.ids[1][1], ROBOT_TYPE.ids[1][2], ROBOT_TYPE.moduleAngleOffsets[1], ROBOT_TYPE.moduleInverts[1]);
                //num // driveID // turnID // turnCANCoderID // turnEncoderOffset // invert
        static SwerveModuleIOSparkMax moduleBL = new SwerveModuleIOSparkMax(//back left
                2, ROBOT_TYPE.ids[2][0], ROBOT_TYPE.ids[2][1], ROBOT_TYPE.ids[2][2], ROBOT_TYPE.moduleAngleOffsets[2], ROBOT_TYPE.moduleInverts[2]);
                //num // driveID // turnID // turnCANCoderID // turnEncoderOffset // invert
        static SwerveModuleIOSparkMax moduleBR = new SwerveModuleIOSparkMax(//back right
                3, ROBOT_TYPE.ids[3][0], ROBOT_TYPE.ids[3][1], ROBOT_TYPE.ids[3][2], ROBOT_TYPE.moduleAngleOffsets[3], ROBOT_TYPE.moduleInverts[3]);
                //num // driveID // turnID // turnCANCoderID // turnEncoderOffset // invert
  }

  public static final class SwerveConstants {
        // These can be safely adjusted without adjusting discrete
        // Some fudge factor is needed for safety while translating + rotating
        // Max speed is 3.4 m/s
        public static final double maxChassisTranslationalSpeed = ModuleConstants.maxFreeWheelSpeedMeters; // Assuming L1 swerve
        public static final double maxWheelLinearVelocityMeters = ModuleConstants.maxFreeWheelSpeedMeters; // Assuming L1 swerve
        public static final double maxChassisAngularVelocity = Math.PI * 1.0;
        
        //distance between swerve modules on x and y axis
        //TODO FIGURE OUT WHAT THE MODULE DISTANCES ARE FOR EACH SWERVE CHASSIS
        public static final double swerveModuleXdistance = Units.inchesToMeters(22); 
        public static final double swerveModuleYdistance = Units.inchesToMeters(22); 
        
        //Module locations in meters
        public static final Translation2d[] moduleLocations = new Translation2d[] {
            new Translation2d( swerveModuleXdistance / 2.0,  swerveModuleYdistance / 2.0),
            new Translation2d( swerveModuleXdistance / 2.0, -swerveModuleYdistance / 2.0),
            new Translation2d(-swerveModuleXdistance / 2.0,  swerveModuleYdistance / 2.0),
            new Translation2d(-swerveModuleXdistance / 2.0, -swerveModuleYdistance / 2.0) };

        // Joystick deadband for no accidental movement
        public static final double deadBand = 0.05;

        //Probs need to update this and move it to RobotType
        public static final double robotMassInKg = 54.43;

        public static final double wheelGripCoefficientOfFriction = 1.19;
    }

    //We use SDS MK4i L1 modules. Google it if you want to verify all this stuff
    public static final class ModuleConstants {

        public static final int driveCurrentLimit = 35;
        public static final int turnCurrentLimit = 20;

        public static final double wheelDiameterMeters = Units.inchesToMeters(4.0); // Assuming SDS module
        
        public static final double driveGearRatio = 8.14; // For SDS MK4i module
        public static final double turnGearRatio = 150.0/7.0; // For SDS MK4i module 21.4285714

        // Both of these measurements should be correct
        // In rotations
        public static final double drivingEncoderPositionFactor = (Math.PI * wheelDiameterMeters) / driveGearRatio;
        
        //This is in m/s not RPM
        public static final double drivingEncoderVelocityPositionFactor = drivingEncoderPositionFactor / 60.0;

        //Rotations per steering rotation for the angle motor need to account for gear ratio
        public static final double turningEncoderPositionFactor = 1 / turnGearRatio;
        public static final double turningEncoderVelocityFactor = turningEncoderPositionFactor / 60;

        // Confirmed working kP!!
        public static final double drivekP = 0.1; // This is good!
        //public static final double drivekP = 0.0;
        public static final double drivekI = 0.0;
        public static final double drivekD = 0.0;

        // See REV: https://motors.vex.com/other-motors/neo
        // The 5790 value is the correct empirical value from the woodblocks
        // Might need to be re-calibrated for carpet or concrete
        public static final double maxRPMWoodBlocks = 5790.0;
        public static final double maxRPMCarpet = 5280.0;

        // Max free speed in RPM originally, converted to RPS native unit
        public static final double maxFreeSpeed = maxRPMCarpet / 60.0;
        // Unit for this: meters/s
        // Calculating it out:
        // 94.6 RPS * pi * 0.1016 m / 8.14 gearing = 3.7094567527 meters / s = 12.1701337 feet / s
        // Therefore, this max wheel free speed works (compared to empirical MK4i free speed)
        public static final double maxFreeWheelSpeedMeters = (maxFreeSpeed * Math.PI * wheelDiameterMeters) / driveGearRatio;
        // Unit for FF: Motor power / meters/s
        // Calculating it out: 1/3.709 = 0.26958125317 power per meters/second
        // If we want to go to the max speed of 3.709, then multiply velocity error by this constant
        // I.e. 3.709 * 0.2695 ~= 1.0

        // DO NOT MODIFY THIS UNLESS YOU KNOW WHAT YOU ARE DOING
        public static final double drivekF = 1.0/maxFreeWheelSpeedMeters;

        public static final double turnkP = 0.7; 
        public static final double turnkI = 0.0;
        public static final double turnkD = 0.0;    
    }

    public static final int PDH_can_id = 15;
    public static boolean fastMode = true;
}
