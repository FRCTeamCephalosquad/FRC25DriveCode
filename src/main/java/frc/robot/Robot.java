// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkMax;

import java.util.HashSet;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the
 * package after creating
 * this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  private final SparkMax m_leftDrive = new SparkMax(2, MotorType.kBrushed);
  private final SparkMax m_rightDrive = new SparkMax(3, MotorType.kBrushed);
  private final SparkMax m_leftDriveFollower = new SparkMax(4, MotorType.kBrushed);
  private final SparkMax m_rightDriveFollower = new SparkMax(5, MotorType.kBrushed);
  public static SparkMax m_coralFeeder = new SparkMax(6, MotorType.kBrushless);

  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftDrive::set, m_rightDrive::set);
  private final XboxController m_controller = new XboxController(0);
  private final Timer m_timer = new Timer();

  AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

  /** Called once at the beginning of the robot program. */
  public Robot() {
    SendableRegistry.addChild(m_robotDrive, m_leftDrive);
    SendableRegistry.addChild(m_robotDrive, m_rightDrive);


    m_robotDrive.setMaxOutput(0.5);

    SparkMaxConfig rightFollowConfig = new SparkMaxConfig();
    rightFollowConfig.follow(m_rightDrive);
    m_rightDriveFollower.configure(rightFollowConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig leftFollowConfig = new SparkMaxConfig();
    leftFollowConfig.follow(m_leftDrive);
    m_leftDriveFollower.configure(leftFollowConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_leftDrive.setInverted(true);

    visionInit();

  }
  Thread m_visionThread;

  void visionInit(){
        // Only vision in robotInit below here
    m_visionThread =
        new Thread(
            () -> {
              var camera = CameraServer.startAutomaticCapture("raw", 0);

              var cameraWidth = 640;
              var cameraHeight = 480;

              camera.setResolution(cameraWidth, cameraHeight);

              var cvSink = CameraServer.getVideo("raw");
              var outputStream = CameraServer.putVideo("RioApriltags", cameraWidth, cameraHeight);

              var mat = new Mat();
              var grayMat = new Mat();

              var pt0 = new Point();
              var pt1 = new Point();
              var pt2 = new Point();
              var pt3 = new Point();
              var center = new Point();
              var red = new Scalar(0, 0, 255);
              var green = new Scalar(0, 255, 0);

              var aprilTagDetector = new AprilTagDetector();

              /* 
              var config = aprilTagDetector.getConfig();
              config.quadSigma = 0.8f;

              config.debug = true;
              aprilTagDetector.setConfig(config);
              

              
              var quadThreshParams = aprilTagDetector.getQuadThresholdParameters();
              quadThreshParams.minClusterPixels = 250;
              quadThreshParams.criticalAngle *= 5; // default is 10
              quadThreshParams.maxLineFitMSE *= 1.5;
              aprilTagDetector.setQuadThresholdParameters(quadThreshParams);
              */
              aprilTagDetector.addFamily("tag36h11", 1);

              var timer = new Timer();
              timer.start();
              var count = 0;

              while (!Thread.interrupted()) {
                if (cvSink.grabFrame(mat) == 0) {
                  outputStream.notifyError(cvSink.getError());
                  continue;
                }

                Imgproc.cvtColor(mat, grayMat, Imgproc.COLOR_RGB2GRAY);
                Imgproc.equalizeHist(grayMat, mat);

                var results = aprilTagDetector.detect(mat);

                var set = new HashSet<>();

                for (var result: results) {
                  count += 1;
                  pt0.x = result.getCornerX(0);
                  pt1.x = result.getCornerX(1);
                  pt2.x = result.getCornerX(2);
                  pt3.x = result.getCornerX(3);

                  pt0.y = result.getCornerY(0);
                  pt1.y = result.getCornerY(1);
                  pt2.y = result.getCornerY(2);
                  pt3.y = result.getCornerY(3);

                  center.x = result.getCenterX();
                  center.y = result.getCenterY();

                  set.add(result.getId());

                  Imgproc.line(mat, pt0, pt1, red, 5);
                  Imgproc.line(mat, pt1, pt2, red, 5);
                  Imgproc.line(mat, pt2, pt3, red, 5);
                  Imgproc.line(mat, pt3, pt0, red, 5);

                  Imgproc.circle(mat, center, 4, green);
                  Imgproc.putText(mat, String.valueOf(result.getId()), pt2, Imgproc.FONT_HERSHEY_SIMPLEX, 2, green, 7);

                };

                for (var id : set){
                  System.out.println("Tag: " + String.valueOf(id));
                }

                if (timer.advanceIfElapsed(1.0)){
                  System.out.println("detections per second: " + String.valueOf(count));
                  count = 0;
                }

                outputStream.putFrame(mat);
              }
              aprilTagDetector.close();
            });
    m_visionThread.setDaemon(true);
    m_visionThread.start();
  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    m_timer.restart();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // Drive for 2 seconds
    if (m_timer.get() < 2.0) {
      // Drive forwards half speed, make sure to turn input squaring off
      m_robotDrive.arcadeDrive(0.5, 0.0, false);
    } else if (m_timer.get() > 2 && m_timer.get() < 2.5) {
      //m_leftDrive.set(-0.1);
      //m_rightDrive.set(0.1);
      m_robotDrive.arcadeDrive(0, .5, false);
    } else if (m_timer.get() > 2.5 && m_timer.get() < 3.5) {
      m_robotDrive.arcadeDrive(0.5, 0.0, false);
    } else {
      m_robotDrive.stopMotor(); // stop robot
    }

    // If time is greater than two but less than three run the coral feeder at 40%
    // speed

    if (m_timer.get() > 3.5 && m_timer.get() < 4.5) {
      m_coralFeeder.set(-0.4);
    } else {
      m_coralFeeder.stopMotor();
    }

  }

  /**
   * This function is called once each time the robot enters teleoperated mode.
   */
  @Override
  public void teleopInit() {
    m_visionThread.interrupt();
  }

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {

    //System.out.println("Yaw Rate: " + gyro.getRawGyroZ());

    m_robotDrive.arcadeDrive(-m_controller.getLeftY(), -m_controller.getRightX());

    if (m_controller.getRightBumperButton()) {
      // do this
      m_coralFeeder.set(-0.4);
    } else {
      // do that
      m_coralFeeder.set(0);
      m_coralFeeder.stopMotor();
    }
  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

}
