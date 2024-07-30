// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import java.lang.annotation.Target;

import org.photonvision.PhotonCamera; //https://maven.photonvision.org/repository/internal/org/photonvision/photonlib-json/1.0/photonlib-json-1.0.json
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;//https://software-metadata.revrobotics.com/REVLib-2024.json

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI; // required for ADIS IMUs
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  private final Spark m_rightDrive = new Spark(1);
  private final Spark m_leftDrive = new Spark(0);
  private final Spark intakeRollerMotor = new Spark(2);

  private CANSparkMax Shooter_R = new CANSparkMax(24, MotorType.kBrushless);
  private CANSparkMax Shooter_L = new CANSparkMax(23, MotorType.kBrushless);
  private CANSparkMax Loader_R = new CANSparkMax(22, MotorType.kBrushless);
  private CANSparkMax Loader_L = new CANSparkMax(21, MotorType.kBrushless);
  private CANSparkMax intakeNeo = new CANSparkMax(3, MotorType.kBrushless);
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftDrive, m_rightDrive);
  private final XboxController m_stick = new XboxController(0);
  private final Timer m_timer = new Timer();
  private final Timer m_noteTimer = new Timer();
  private final Encoder right_encoder = new Encoder(6, 7, true, CounterBase.EncodingType.k4X);
  private final Encoder left_encoder = new Encoder(8, 9, false, CounterBase.EncodingType.k4X);
  private final Field2d m_field = new Field2d();
  public static final ADIS16470_IMU imu = new ADIS16470_IMU(ADIS16470_IMU.IMUAxis.kY, ADIS16470_IMU.IMUAxis.kX, ADIS16470_IMU.IMUAxis.kZ, SPI.Port.kOnboardCS0, ADIS16470_IMU.CalibrationTime._1s);
  PhotonCamera noteCamera = new PhotonCamera("AVerMedia_PW315");
  PhotonCamera tagCamera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
  public double SpeedForward, rotationClockwise, cYaw, tYaw, turnToNote, noteTargetYaw;
  public static boolean pHasTarget, noteTarget, tagTarget, getNote;
  public static double intakeSpeed = 0, MaxIntakeSpeed = 1, loaderSpeed = 0, noteDistance, MaxTurnEffort = 0.6, MaxSpeed = 1;
  public static int notePickupPhase = 1;
  public AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  public Transform3d cameraToRobot = new Transform3d(0, 0, 0.75, new Rotation3d(0, 330, 0)); 
    // From Robot Intake: (Forwards[x]:, Left[y]:, up[z]:, Camera Rotation)
  public Pose2d robotPosition = new Pose2d();

  @Override
  public void robotInit() {
    imu.calibrate();
    m_rightDrive.setInverted(true);
    right_encoder.setSamplesToAverage(5);
    left_encoder.setSamplesToAverage(5);
    right_encoder.setDistancePerPulse(10.0 * 0.3048 / 12928); // pulses averaged from l & r = 12928 of 10' (5 x 2' tiles)
    left_encoder.setDistancePerPulse(10.0 * 0.3048 / 12928); // 0.3048 converts feet into metres
    right_encoder.setMinRate(0.01);
    left_encoder.setMinRate(0.01);
    right_encoder.reset();
    left_encoder.reset();
    SmartDashboard.putNumber("MaxIntakeSpeed", MaxIntakeSpeed);
    SmartDashboard.putNumber("MaxSpeed", MaxSpeed);
    m_timer.reset();
    m_noteTimer.reset();
    m_timer.start();
  }

  @Override
  public void robotPeriodic() {
    cYaw = -imu.getAngle(ADIS16470_IMU.IMUAxis.kY) % 360;

    var noteResult = noteCamera.getLatestResult();
    var tagResult = tagCamera.getLatestResult();
    noteTarget = noteResult.hasTargets();
    tagTarget = tagResult.hasTargets();

    if (noteTarget) {
      turnToNote = noteResult.getBestTarget().getYaw();
      noteTargetYaw = (cYaw + turnToNote) % 360;
      double noteDistance = PhotonUtils.calculateDistanceToTargetMeters(0.5, 0, 0, Units.degreesToRadians(noteResult.getBestTarget().getPitch())); 
        // (Camera Heght, Target Height, Camera Pitch)
      SmartDashboard.putNumber("Note Camera Angle", turnToNote);
      SmartDashboard.putNumber("Note Target Angle", noteTargetYaw);
      SmartDashboard.putNumber("Note Distance", noteDistance);} 
    
    if (tagTarget) {
      PhotonTrackedTarget target = tagResult.getBestTarget();
      Pose3d tagPose = aprilTagFieldLayout.getTagPose(target.getFiducialId()).get();
      // Transform3d cameraToRobot = {0.5, 0.5, 0.5};
      Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(
        target.getBestCameraToTarget(), 
        tagPose, 
        cameraToRobot);
      // Pose2d robotPosition = new Pose2d(robotPose.getX(), robotPose.getY(), robotPose.getRotation().toRotation2d());
      Pose2d robotPosition = new Pose2d(robotPose.getTranslation().toTranslation2d(), robotPose.getRotation().toRotation2d());
      m_field.setRobotPose(robotPosition);
      SmartDashboard.putData("robotPose", m_field);
    }
    // var result = noteCamera.getLatestResult();
    // noteTarget = result.hasTargets();
    // SmartDashboard.putBoolean("Has Target", noteTarget);
    // if (noteTarget) {
    //   pTargetYaw = result.getBestTarget().getYaw();
    // //consider an additional conditional to make sure the target for tYaw is
    // tYaw = cYaw - pTargetYaw;
    // SmartDashboard.putNumber("TargetID", result.getBestTarget().getFiducialId());
    // SmartDashboard.putNumber("Target Yaw", pTargetYaw);
    // }

    SmartDashboard.putNumber("Angle", cYaw);
    SmartDashboard.putNumber("R_Encoder Distance", right_encoder.getDistance());
    SmartDashboard.putNumber("L_Encoder Distance", left_encoder.getDistance());
    SmartDashboard.putNumber("R_Encoder", right_encoder.get());
    SmartDashboard.putNumber("L_Encoder", left_encoder.get());
    SmartDashboard.putNumber("Rotation Clockwise", rotationClockwise);
    SmartDashboard.putNumber("SpeedForwad", SpeedForward);
    SmartDashboard.putBoolean("Note", noteTarget);

    SmartDashboard.getNumber("MaxSpeed", MaxSpeed);
  }

  @Override
  public void autonomousInit() {
    // m_timer.reset();
    // right_encoder.reset();
    // left_encoder.reset();
    // m_timer.start();
    // imu.reset();
  }

  @Override
  public void autonomousPeriodic() {
    // intakeNeo.set(0.25);
    // intakeRollerMotor.set(-1);
    // if (m_timer.get() >= 0 && m_timer.get() <= 2) {
    //   Shooter_L.set(1);
    //   Shooter_R.set(-1);
    // } else if (m_timer.get() >= 2 && m_timer.get() <= 4) {
    //   Loader_L.set(0.8);
    //   Loader_R.set(-0.8);
    // } else if (m_timer.get() >= 4 && m_timer.get() <= 7) {
    //   m_robotDrive.arcadeDrive(-0.6, 0);
    //   Loader_L.set(-0.05);
    //   Loader_R.set(-0.05);
    //   Shooter_L.set(-0.1);
    //   Shooter_R.set(-0.1);
    // } else if (m_timer.get() >= 7.5 && m_timer.get() <= 10) {
    //   m_robotDrive.arcadeDrive(0.71, 0);
    // } else if (m_timer.get() >= 10 && m_timer.get() <= 12) {
    //   Shooter_L.set(1);
    //   Shooter_R.set(-1);
    // } else if (m_timer.get() >= 12 && m_timer.get() <= 14) {
    //   Loader_L.set(0.6);
    //   Loader_R.set(-0.6);
    // } else {
    //   Shooter_L.set(0);
    //   Shooter_R.set(0);
    //   Loader_L.set(0);
    //   Loader_R.set(0);
    //   intakeNeo.set(0);
    //   intakeRollerMotor.set(0);
    // }
  }

  @Override
    public void teleopInit() {
      notePickupPhase = 0;
    }

  @Override
  public void teleopPeriodic() {
    if (m_stick.getLeftBumper()) {
      right_encoder.reset();
      left_encoder.reset();
    }

    if (m_stick.getRightBumper()) {
      imu.reset();
    }

    if (m_stick.getPOV() == 0 && noteTarget) {
      getNote = true;
      notePickupPhase = 1;
  } else if (m_stick.getBackButtonPressed()) {
      getNote = false;
  } else if (m_stick.getStartButtonPressed()) {
      notePickupPhase = 6;
  }
    

    // forward & backwards movement
    SpeedForward = (m_stick.getRightTriggerAxis() - m_stick.getLeftTriggerAxis());
    
    // rotation
    rotationClockwise = -m_stick.getLeftX();

    // intake forward & backwards
    intakeSpeed = m_stick.getAButton() ? MaxIntakeSpeed : 0;
    intakeSpeed = m_stick.getYButton() ? -MaxIntakeSpeed : intakeSpeed;

    if (getNote) {
      switch (notePickupPhase) {
        case 1: // Rotate to Note
          rotationClockwise = Math.pow((1 + turnToNote / 10), 2) * Math.signum(turnToNote);
          notePickupPhase = turnToNote < 5 && turnToNote > - 5 ? 2 : 1;
          break;
        
        case 2: // Drive Forward
          SpeedForward = 1 - Math.pow(1 + (1 / noteDistance), 2);
          notePickupPhase = noteTarget ? 2 : 3;
          break;
        
        case 3: // Timer Start
          m_noteTimer.reset();
          m_noteTimer.start();
          notePickupPhase = 4;
        
        case 4: // Drie Forward and Intake
          SpeedForward = 0.3;
          intakeSpeed = MaxIntakeSpeed;
          notePickupPhase = m_noteTimer.get() <= 3 ? 4 : 5;
          break;
        
        case 5: // Continue Intake
          intakeSpeed = MaxIntakeSpeed;
          notePickupPhase = m_noteTimer.get() <= 4 ? 5 : 0;
          break;

        case 6: // Manual Reverse
          SpeedForward = -0.3;
          intakeSpeed = -MaxIntakeSpeed;
          break;
        }

    }

    // set methods
    Loader_L.set(loaderSpeed);
    Loader_R.set(-loaderSpeed);
    Shooter_L.set(0);
    Shooter_R.set(0);

    // shoot
    if (m_stick.getBButton()) {
      if (m_timer.get() >= 0 && m_timer.get() <= 2.6) {
        Shooter_L.set(1);
        Shooter_R.set(-1);
      }
      if (m_timer.get() >= 2 && m_timer.get() <= 4) {
        Loader_L.set(0.3);
        Loader_R.set(-0.3);
      }
    } else {
      m_timer.reset();
    }

    intakeNeo.set(intakeSpeed * 0.25);
    intakeRollerMotor.set(-intakeSpeed);

    rotationClockwise = rotationClockwise > 1 ? 1 : rotationClockwise;
    SpeedForward = SpeedForward > 1 ? 1 : SpeedForward;

    m_robotDrive.arcadeDrive(SpeedForward * MaxSpeed, rotationClockwise * MaxTurnEffort * MaxSpeed, false);
  }
}
