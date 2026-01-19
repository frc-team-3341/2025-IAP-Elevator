package frc.robot.subsystems;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.hal.JNIWrapper;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.util.SwerveUtil;

/**
 * <p>
 * Creates a SwerveDrive class.
 * </p>
 * 
 * <p>
 * In its current form, it can be simulated using the simulation integration
 * method from the static SwerveUtil class. This simulation is less precise than
 * real life, but much better than AutoDesk Synthesis :).
 * </p>
 * 
 * @author Aric Volman
 */
public class SwerveDriveTrain extends SubsystemBase {

   private boolean fieldRelative = true;

   // Create Navx
   private AHRS navx = new AHRS(NavXComType.kMXP_SPI);


   // Create object representing swerve modules
   private SwerveModuleIOSparkMax[] moduleIO;

   // Create object that represents swerve module positions (i.e. radians and
   // meters)
   private SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];

   // Create kinematics object
   private SwerveDriveKinematics kinematics;

   private ChassisSpeeds chassisSpeeds;

   // Create poseEstimator object
   // This can fuse Visual and Encoder odometry with different standard
   // deviations/priorities
   private SwerveDrivePoseEstimator poseEstimator;

   // Add field to show robot
   private Field2d field;
   private Rotation2d offsetNavx = Rotation2d.fromDegrees(0);
   private final StructArrayPublisher<SwerveModuleState> statePublisher;
   private final StructArrayPublisher<SwerveModuleState> targetStatePublisher;
   private final StructArrayPublisher<SwerveModuleState> absStatePublisher;
   private final StructPublisher<ChassisSpeeds> chassisSpeedsPublisher;
   private final StructPublisher<Pose2d> poseEstimatorPublisher;


   /**
    * Creates a new SwerveDrive object. Intended to work both with real modules and
    * simulation.
    * 
    * @author Aric Volman
    */
   public SwerveDriveTrain(Pose2d startingPose, SwerveModuleIOSparkMax FL, SwerveModuleIOSparkMax FR, SwerveModuleIOSparkMax BL, SwerveModuleIOSparkMax BR) {
      // Assign modules to their object
      this.moduleIO = new SwerveModuleIOSparkMax[] { FL, FR, BL, BR};

      // Iterate through module positions and assign initial values
      modulePositions = SwerveUtil.setModulePositions(moduleIO);

      // Initialize all other objects
      this.kinematics = new SwerveDriveKinematics(Constants.SwerveConstants.moduleLocations);
      // Can set any robot pose here (x, y, theta) -> Built in Kalman Filter
      // FUTURE: Seed pose with CV
      // Auto is field-oriented
      this.poseEstimator = new SwerveDrivePoseEstimator(this.kinematics, Rotation2d.fromDegrees(getGyroYaw()),
            this.modulePositions, startingPose);
      this.field = new Field2d();
      
      this.chassisSpeeds =  new ChassisSpeeds(0.0, 0.0, 0.0);
      statePublisher = NetworkTableInstance.getDefault().getStructArrayTopic("/SwerveStates", SwerveModuleState.struct).publish();
      absStatePublisher = NetworkTableInstance.getDefault().getStructArrayTopic("/SwerveStates_abs", SwerveModuleState.struct).publish();
      targetStatePublisher = NetworkTableInstance.getDefault().getStructArrayTopic("/SwerveStates_target", SwerveModuleState.struct).publish();
      chassisSpeedsPublisher = NetworkTableInstance.getDefault().getStructTopic("/ChassisSpeeds", ChassisSpeeds.struct).publish();
      poseEstimatorPublisher = NetworkTableInstance.getDefault().getStructTopic("/EstimatedPose", Pose2d.struct).publish();

      //Make sure to call this last since we want everything else to be configured
      
   }


   public Command getAutonomousCommand() {
      return null;
   }

   public void periodic() {
      SmartDashboard.putBoolean("Field Relative", this.fieldRelative);

      // Update module positions
      
      //Update pose using gyro and encoders.
      this.poseEstimator.update(this.getRotation(), this.modulePositions);
      
      poseEstimatorPublisher.set(poseEstimator.getEstimatedPosition());

      this.field.setRobotPose(this.getPoseFromEstimator());

      // Update telemetry of each swerve module

      // Put field on SmartDashboard
      SmartDashboard.putData("Field", this.field);
      SmartDashboard.putNumber("Robot Rotation", getPoseFromEstimator().getRotation().getDegrees());
      SmartDashboard.putNumber("Angle", getHeading());

      
      SmartDashboard.putBoolean("is calibartig", navx.isCalibrating());
 
      SmartDashboard.putNumber("offsetNavx", offsetNavx.getDegrees());
      //SmartDashboard.putNumber("pose.getRotation()", pose.getRotation().getDegrees());
      SmartDashboard.putNumber("navx.getRotation2d", getHeading());

      SmartDashboard.putNumber("Match Time", Timer.getMatchTime());

      targetStatePublisher.set(getSetpointStates());
      statePublisher.set(getActualStates());
      absStatePublisher.set(getCanCoderStates());
      chassisSpeedsPublisher.set(this.chassisSpeeds);

   }

   public void simulationPeriodic() {
      // Add simulation! Yes, with the Util class, it's that easy!
      // WARNING: This doesn't use the Navx, just the states of the 
   }

   // command toggle field centric
   public Command toggleFieldCentric() {
      return this.runOnce(() -> {
         this.fieldRelative = !this.fieldRelative;
      });
   }

   /**
    * Drive either field oriented, or not field oriented
    * 
    * @param translation   Vector of x-y velocity in m/s
    * @param rotation      Rotation psuedovector in rad/s
    * @param isOpenLoop    Whether or not to control robot with closed or open loop
    *                      control
    */
    public void drive(Translation2d translation, double rotation, boolean isOpenLoop) {
      //This question mark and colon are called ternary operators
      //If field relative is true, then do the line with the ?, if false do :
      this.chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation,
                  this.getRotation());

      this.chassisSpeeds = SwerveUtil.discretize(this.chassisSpeeds, -4.0);

      // Convert the robot vector into module states which is a vector for each module
      // Explanation found here
      // https://samliu.dev/blog/a-deep-dive-into-swerve#16d4e0ca3f0280b19d85cdb8b2adac83
      SwerveModuleState[] swerveModuleStates = this.kinematics.toSwerveModuleStates(this.chassisSpeeds);

      // MUST USE SECOND TYPE OF METHOD
      SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, this.chassisSpeeds,
            Constants.SwerveConstants.maxWheelLinearVelocityMeters,
            Constants.SwerveConstants.maxChassisTranslationalSpeed,
            Constants.SwerveConstants.maxChassisAngularVelocity);

      for (int i = 0; i < swerveModuleStates.length; i++) {
         this.moduleIO[i].setDesiredState(swerveModuleStates[i]);
      }
   }

   /**
    * Gets the SwerveModuleState[] for our use in code.
    */
   public SwerveModuleState[] getSetpointStates() {
      SwerveModuleState[] states = new SwerveModuleState[moduleIO.length];
      for (int i = 0; i < states.length; i++) {
         states[i] = this.moduleIO[i].getDesiredState();
      }
      return states;
   }

   /**
    * Gets the actual SwerveModuleState[] for our use in code
    */
   public SwerveModuleState[] getActualStates() {
      SwerveModuleState[] states = new SwerveModuleState[moduleIO.length];
      for (int i = 0; i < states.length; i++) {
         states[i] = this.moduleIO[i].getActualModuleState();
      }
      return states;
   }

   public SwerveModuleState[] getCanCoderStates() {
      SwerveModuleState[] states = new SwerveModuleState[moduleIO.length];
      for (int i = 0; i < states.length; i++) {
         states[i] = this.moduleIO[i].getCanCoderState();
      }
      return states;
   }

   /**
    * Sets the velocities and positions (drive, turn) of one module
    * 
    * @param driveVel Drive velocity (m/s)
    * @param turnPos  Turn position (degrees)
    * @param index    Index of module
    */
   public void setModuleSetpoints(double driveVel, double turnPos, int index) {
      // Precondition: Safety check within bounds
      if (index >= 0 && index < moduleIO.length) {
         SwerveModuleState state = new SwerveModuleState(driveVel, Rotation2d.fromDegrees(turnPos));
         moduleIO[index].setDesiredState(state);
      }
   }

   /**
    * Stops the motors of the swerve drive. Useful for stopping all sorts of
    * Commands.
    */
   public void stopMotors() {
      for (SwerveModuleIOSparkMax module : moduleIO) {
         module.setDriveVoltage(0.0);
         module.setTurnVoltage(0.0);
      }
   }

   // Returns the current yaw value (in degrees, from -180 to 180)
   public double getGyroYaw() {
      return navx.getYaw();
   }

   /** 
    * Get heading of Navx. Negative because Navx is CW positive.
    */
    public double getHeading() {
      return -navx.getRotation2d().plus(offsetNavx).getDegrees();
   }

   /** 
    * Get rate of rotation of Navx. Negative because Navx is CW positive.
    */
   public double getTurnRate() {
      return -navx.getRate();
   }

   /** 
    * Get Rotation2d of Navx. Positive value (CCW positive default).
    */
   public Rotation2d getRotation() {
      return navx.getRotation2d().plus(offsetNavx);
   }


   /**
    * Get Pose2d of poseEstimator.
    */
   public Pose2d getPoseFromEstimator() {
      return poseEstimator.getEstimatedPosition();
   }

   /**
    * Reset pose of robot to pose
    */
   public void resetPose(Pose2d pose) {
      System.out.println("resetPose");
      poseEstimator.resetPosition(pose.getRotation(), modulePositions, pose);
      // offsetNavx = pose.getRotation().minus(navx.getRotation2d());

      
   }

   /**
    * Get chassis speeds for PathPlannerLib
    */
   public ChassisSpeeds getRobotRelativeSpeeds() {
      return ChassisSpeeds.fromFieldRelativeSpeeds(kinematics.toChassisSpeeds(getActualStates()), getRotation());
   }

   /** Gets field */
   public Field2d getField() {
      return field;
   }

   public void setModulesPositions(double velocity, double angle) {
      for (int i = 0; i < 4; i++) {
         setModuleSetpoints(velocity, angle, i);
      }
   }

   public Command resetHeadingCommand() {
      return runOnce(() -> {
         navx.reset();
      });
   }

   

}