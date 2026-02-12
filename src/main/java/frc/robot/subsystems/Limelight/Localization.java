package frc.robot.subsystems.Limelight;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Swerve.DriveSubsystem;
import frc.robot.utils.SmartDashboardNumber;

public class Localization extends SubsystemBase {
    
  // Poses (x, y, angle) for scoring targets, the robot's current pose, and mechanism offsets 
  public static final Pose2d exampleTarget = new Pose2d(0, 0, new Rotation2d());
  public static final Pose2d exampleMechanismtOffset = new Pose2d(-0.22, 0.25, new Rotation2d());
    
  // Valid IDs for Tracking - Remove undesired tags from array
  private static int[] validIDs = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32};

  /*
   * Tag Guide (Driver Station Perspective, Hub Face Perspective):
   * 1: Red Trench Right (Neutral)
   * 2: Red Hub Right (Right)
   * 3: Red Hub Back (Left)
   * 4: Red Hub Back (Right)
   * 5: Red Hub Left (Left)
   * 6: Red Trench Left (Neutral)
   * 7: Red Trench Left (Alliance)
   * 8: Red Hub Left (Right)
   * 9: Red Hub Front (Left)
   * 10: Red Hub Front (Right)
   * 11: Red Hub Right (Left)
   * 12: Red Trench Right (Alliance)
   * 13: Red Outpost (Right)
   * 14: Red Outpost (Left)
   * 15: Red Tower (Right)
   * 16: Red Tower (Left)
   * 17: Blue Trench Right (Neutral)
   * 18: Blue Hub Right (Right)
   * 19: Blue Hub Back (Left)
   * 20: Blue Hub Back (Right)
   * 21: Blue Hub Left (Left)
   * 22: Blue Trench Left (Neutral)
   * 23: Blue Trench Left (Alliance)
   * 24: Blue Hub Left (Right)
   * 25: Blue Hub Front (Left)
   * 26: Blue Hub Front (Right)
   * 27: Blue Hub Right (Left)
   * 28: Blue Trench Right (Alliance)
   * 29: Blue Outpost (Right)
   * 30: Blue Outpost (Left)
   * 31: Blue Tower (Right)
   * 32: Blue Tower (Left)
   */

  // Limelights
  private static String[] limelightNames = {
    VisionConstants.kLimelightFrontLeftName,
    VisionConstants.kLimelightFrontRightName,
    VisionConstants.kLimelightBackLeftName,
    VisionConstants.kLimelightBackRightName
  };

  // The X, Y, Theta standard deviations for each LL
  private static double[][] limelightStdvs = {
    {0.8, 0.8, 9999},
    {0.8, 0.8, 9999},
    {0.8, 0.8, 9999},
    {0.8, 0.8, 9999}
  };

  // Array of the Estimator Wrappers
  private static LimelightPoseEstimateWrapper[] wrappers;

  // Denominator for the stdv calculation
  private static SmartDashboardNumber stdvDenominator = new SmartDashboardNumber("Stdv Denominator Scale", 30);

  // Initializes the each of the Limelights on the robots into the wrapper
  public static void initialize() {

    // Don't initialize if there's stuff already
    if (wrappers != null) {
      return;
    }
    
    // Wrapper becomes of length equal to Limelights on robot
    wrappers = new LimelightPoseEstimateWrapper[limelightNames.length];

    // Add each limelight to the wrapper array
    for (int i = 0; i < limelightNames.length; i++) {

      // Init LL
      wrappers[i] = new LimelightPoseEstimateWrapper().withName(limelightNames[i]);

      // Adds condition that filters out undesired IDs
      LimelightHelpers.SetFiducialIDFiltersOverride(limelightNames[i], validIDs);
    }
  }

  // Gets the pose estimates for each Limelight
  public static LimelightPoseEstimateWrapper[] getPoseEstimates(double headingDegrees) {
    
    // Make sure LLs are initialized and ready to go
    if (wrappers == null) {
      initialize();
    }
    
    // Updates wrapper variables for each Limelight
    for (int i = 0; i < limelightNames.length; i++) {
      String name = limelightNames[i];
      LimelightHelpers.SetRobotOrientation(name, headingDegrees + DriveSubsystem.AllianceYaw, RobotContainer.m_robotDrive.getTurnRate(), 0, 0, 0, 0);
      wrappers[i].withPoseEstimate(LimelightHelpers.getBotPoseEstimate_wpiBlue(name))
        .withTagInVision(LimelightHelpers.getTV(name));
    }
    return wrappers;
  }

  // EXAMPLE CODE for getDistance()
  public static double getDistanceToExampleTarget() {
    return Math.hypot(getLimelightPose2d().getX() - exampleTarget.getX(), getLimelightPose2d().getY() - exampleTarget.getY());
  }

  // EXAMPLE Code for getAngle()
  public static Rotation2d getAngleToExampleTarget() {
    return Rotation2d.fromRadians(
      Math.atan2(exampleTarget.getY() - getLimelightPose2d().getY(), exampleTarget.getX() - getLimelightPose2d().getX())
    );
  }

  // Returns Pose2d
  public static Pose2d getPose2d() { 
    return RobotContainer.m_robotDrive.getPose();
  }

  // Finds a mechanism's (e.g. turret's) field relative position
  public static Pose2d getLimelightPose2d() {

    // Gets current bot position
    Pose2d botPose = getPose2d();
    double theta = botPose.getRotation().getRadians();

    // Add matrix tranformation to robot pose to get field-centric mechanism pose
    return new Pose2d(
      botPose.getX() + exampleMechanismtOffset.getX() * Math.cos(theta) - exampleMechanismtOffset.getY() * Math.sin(theta),
      botPose.getY() + exampleMechanismtOffset.getX() * Math.sin(theta) + exampleMechanismtOffset.getY() * Math.cos(theta),
      new Rotation2d()
    );
  }

  // A wrapper that will make life much easier to handle LL 
  public static class LimelightPoseEstimateWrapper {

    // Pose Estimate
    public LimelightHelpers.PoseEstimate poseEstimate;

    // Name of Current LL
    public String limelightName;

    // tiv stands for "tag in vision"
    public boolean tiv;

    // For Telemetry
    private SmartDashboardNumber[] stdvs = new SmartDashboardNumber[3];

    // For Field Visualization in Shuffleboard
    public Field2d field = new Field2d();

    // Adjusts each stdv based on distance to target
    public Matrix<N3, N1> getStdvs(double distanceToTarget) {
      return VecBuilder.fill(
        adjustStdv(stdvs[0].getNumber(), distanceToTarget),
        adjustStdv(stdvs[1].getNumber(), distanceToTarget),
        adjustStdv(stdvs[2].getNumber(), distanceToTarget)
      );
    }

    // Updates the stdvs for a Limelight 
    public LimelightPoseEstimateWrapper withName(String name) {

      // Updates the local name variable in the Limelight wrapper
      this.limelightName = name;
      
      // Default standard deviation values
      double[] stdvDefVals = new double[] {0.8, 0.8, 9999};

      // For the specific limelight, update stdvs
      for (int i = 0; i < Localization.limelightNames.length; i++) {

        // Override the default values with updated values if available
        if (Localization.limelightNames[i].equals(name)) {
          stdvDefVals = limelightStdvs[i];
          break;
        }
      }

      // Updates the numbers to SmartDashboard
      stdvs[0] = new SmartDashboardNumber(this.limelightName + "/" + this.limelightName + "-stdvX", stdvDefVals[0]);
      stdvs[1] = new SmartDashboardNumber(this.limelightName + "/" + this.limelightName + "-stdvY", stdvDefVals[1]);
      stdvs[2] = new SmartDashboardNumber(this.limelightName + "/" + this.limelightName + "-stdvTheta", stdvDefVals[2]);

      // Puts the field map on SmartDashboard
      // SmartDashboard.putData(this.limelightName + "/" + this.limelightName + "-field", this.field);

      // Returns this to chain commands
      return this;
    }

    // Updates the local poseEstimate variable in the Limelight wrapper
    public LimelightPoseEstimateWrapper withPoseEstimate(LimelightHelpers.PoseEstimate estimate) {
      this.poseEstimate = estimate;
      return this;
    }

    // Updates the tiv variable in the Limelight wrapper
    public LimelightPoseEstimateWrapper withTagInVision(boolean b) {
      this.tiv = b;
      SmartDashboard.putBoolean(this.limelightName + "/" + this.limelightName + "-tag-in-vision", b);
      return this;
    }

    // Adjusts standard deviation based on target distance
    private double adjustStdv(double stdv, double distanceToTarget) {
      return stdv + stdv * (distanceToTarget * distanceToTarget) / Localization.stdvDenominator.getNumber();
    }
  }
}


