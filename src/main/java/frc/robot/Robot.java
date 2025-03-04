// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the
 * package after creating
 * this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  // Drive Motors
  private final SparkMax m_leftDrive = new SparkMax(2, MotorType.kBrushed);
  private final SparkMax m_rightDrive = new SparkMax(3, MotorType.kBrushed);
  private final SparkMax m_leftDriveFollower = new SparkMax(4, MotorType.kBrushed);
  private final SparkMax m_rightDriveFollower = new SparkMax(5, MotorType.kBrushed);

  // Drive Control
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftDrive::set, m_rightDrive::set);
  private final XboxController m_controller = new XboxController(0);

  // Coral Motor
  private final SparkMax m_coralFeeder = new SparkMax(6, MotorType.kBrushless);

  // Navigation Aids
  private final AHRS gyro = new AHRS(NavXComType.kUSB1);
  private final Vision vision = new Vision();

  /**
   * Set up the robot, this is called ONCE when the robot code starts
   */
  public Robot() {
    SendableRegistry.addChild(m_robotDrive, m_leftDrive);
    SendableRegistry.addChild(m_robotDrive, m_rightDrive);

    // Set up right follower
    SparkMaxConfig rightFollowConfig = new SparkMaxConfig();
    rightFollowConfig.follow(m_rightDrive);
    m_rightDriveFollower.configure(rightFollowConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Set up left follower
    SparkMaxConfig leftFollowConfig = new SparkMaxConfig();
    leftFollowConfig.follow(m_leftDrive);
    m_leftDriveFollower.configure(leftFollowConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_leftDrive.setInverted(true);

    vision.start();

    autonomousOptionSetup();
  }

  /**
   * This is called over and over again while the robot is running.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  /**
   * Drive for the given amount of time, at the given speed
   */
  Command driveForTime(double time, double speed) {
    return Commands.sequence(
        new RunCommand(() -> m_robotDrive.arcadeDrive(speed, 0.0, false)).raceWith(new WaitCommand(time)),
        new InstantCommand(() -> m_robotDrive.arcadeDrive(0, 0.0, false)));
  }

  /**
   * Turn for the given amount of time, at the given speed
   */
  Command turnForTime(double time, double turn) {
    return Commands.sequence(
        new RunCommand(() -> m_robotDrive.arcadeDrive(0, turn, false)).raceWith(new WaitCommand(time)),
        new InstantCommand(() -> m_robotDrive.arcadeDrive(0, 0.0, false)));
  }

  /**
   * YEET that coral
   */
  Command coralYeeter() {
    return Commands.sequence(
        new RunCommand(() -> m_coralFeeder.set(-0.2)).raceWith(new WaitCommand(1)),
        new InstantCommand(() -> m_coralFeeder.stopMotor()));
  }

  /**
   * Do nothing
   */
  Command park() {
    return new RunCommand(() -> m_robotDrive.stopMotor());
  }

  /*
   * Turn the robot by a number of degrees, using the gyro
   */
  Command turnDegrees(double degrees) {
    final double SLOW_TURN = 0.6; // When we are close go this fast
    final double FAST_TURN = 0.7; // If we are further, go this fast

    // gyro Positive is right
    // Gyro jumps from 180 to -180
    return new Command() {
      boolean done = false;

      /**
       * When this command starts it sets the control deadband to
       * zero, and resets the gyro yaw to zero.
       */
      @Override
      public void initialize() {
        m_robotDrive.setDeadband(0);
        gyro.zeroYaw();
      }

      @Override
      public void execute() {
        // How far are we from the direction we want to go?
        double error = degrees - gyro.getYaw();

        // Figure out which way to turn
        double turn = -Math.signum(error);

        // And how fast
        if (Math.abs(error) < 45)
          turn *= SLOW_TURN;
        else
          turn *= FAST_TURN;

        // Set the turn
        m_robotDrive.arcadeDrive(0, turn, false);

        // If we are very close, set done to true.
        if (Math.abs(error) < 10) {
          done = true;
        }

      }

      /*
       * Is this command finished?
       */
      @Override
      public boolean isFinished() {
        return done;
      }

      /**
       * When done turn off motors and reset deadband
       */
      @Override
      public void end(boolean interrupted) {
        m_robotDrive.stopMotor();
        m_robotDrive.setDeadband(RobotDriveBase.kDefaultDeadband);
      }

    };
  }

  /**
   * This command drives towards an april tag.
   * 
   * It will just drive forward slowly no matter what, once it sees
   * an april tag it will start steering towards it.
   * 
   * @param tagId
   * @return
   */
  Command seekAprilTagAhead(int tagId) {

    final double SLOW_TURN = 0.3; // Speed to turn when it is almost straight ahead
    final double FAST_TURN = 0.5; // Speed to turn if it is off to the side

    final double TAG_SPEED = 0.5; // Speed to drive forward when we see it
    final double SEARCH_SPEED = 0.4; // Speed to drive forward when we do not see it

    return new Command() {
      @Override
      public void initialize() {
        m_robotDrive.setDeadband(0);
        vision.lookFor(tagId);
      }

      @Override
      public void execute() {

        if (vision.isTargetInSight()) {
          System.out.println("April Tag in sight: " + vision.targetPosition());
          double targetOffset = -vision.targetPosition();
          double turn = 0;
          if (targetOffset > .3) {
            turn = FAST_TURN;
          } else if (targetOffset < -.3) {
            turn = -FAST_TURN;
          } else if (targetOffset > .1) {
            turn = SLOW_TURN;
          } else if (targetOffset < -.1) {
            turn = -SLOW_TURN;
          }

          m_robotDrive.arcadeDrive(TAG_SPEED, turn, false);
        } else {
          System.out.println("April tag not in sight.");
          m_robotDrive.arcadeDrive(SEARCH_SPEED, 0, false);
        }
      }

      @Override
      public void end(boolean interrupted) {
        m_robotDrive.setDeadband(RobotDriveBase.kDefaultDeadband);
        m_robotDrive.stopMotor();
        vision.lookFor(0);
      }
    };
  }

  private static final String autoDefault = "Drive Off Line";
  private static final String autoRedRight = "Red Right (Reef) 11";
  private static final String autoRedMiddle = "Red Middle 10";
  private static final String autoRedLeft = "Red Left 4";

  private static final String autoBlueRight = "Blue Right (Reef) 20";
  private static final String autoBlueMiddle = "Blue Middle 21";
  private static final String autoBlueLeft = "Blue Left 22";

  public void autonomousOptionSetup() {
    SmartDashboard.putStringArray("Auto List", new String[] {
        autoDefault,
        autoRedRight,
        autoRedMiddle,
        autoRedLeft,
        autoBlueRight,
        autoBlueMiddle,
        autoBlueLeft
    });
  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {

    String autoName = SmartDashboard.getString("Auto Selector", autoDefault);
    System.out.println("RUNNING AUTONOMOUS " + autoName);

    CommandScheduler.getInstance().schedule(//
        Commands.sequence(
            driveForTime(2.2, 0.5),             //Drive Forward
            turnDegrees(-45),                   //Turn Left 45 degrees
            seekAprilTagAhead(11)//             //Seek April Tag 11
                .raceWith(new WaitCommand(5)),    //For up to 5 seconds
            new WaitCommand(1),                 //Wait a second
            coralYeeter(),                      //YEET
            new WaitCommand(1),                 //Wait a second
            driveForTime(1, -.5),               //Back up
            park()                              //Stop movement
        ));


  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void autonomousExit() {
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called once each time the robot enters teleoperated mode.
   */
  @Override
  public void teleopInit() {
  }

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {

    // Drive Code

    // Left trigger = slow speed
    if (m_controller.getLeftTriggerAxis() > 0.9) {
      m_robotDrive.setMaxOutput(0.4);
    } else {
      m_robotDrive.setMaxOutput(0.6);
    }

    // Get forward speed
    double forwardSpeed = -m_controller.getLeftY();
    // Left is positive
    double rotateSpeed = -m_controller.getRightX();

    m_robotDrive.setDeadband(RobotDriveBase.kDefaultDeadband);
    m_robotDrive.arcadeDrive(forwardSpeed, rotateSpeed);

    // Coral YEET Teleop code
    if (m_controller.getRightBumperButton()) {
      // Eject coral into level 1
      m_coralFeeder.set(-0.25);
    } else if (m_controller.getLeftBumperButton()) {
      // TEST eject coral to level 2?
      // m_coralFeeder.set(0.5);
    } else {
      // stop
      m_coralFeeder.set(0);
      m_coralFeeder.stopMotor();
    }
  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {
    m_robotDrive.setDeadband(0);
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    SmartDashboard.putBoolean("DB/LED 0", gyro.isConnected());
    SmartDashboard.putString("DB/String 0", gyro.getYaw() + "");

    if (m_controller.getPOV() == 0) {
      // m_robotDrive.arcadeDrive(0.6, 0, false);
    } else if (m_controller.getYButton()) {
      m_robotDrive.setDeadband(0);
      if (vision.isTargetInSight()) {
        System.out.println(vision.targetPosition());
        double targetOffset = -vision.targetPosition();
        double turn = 0;
        if (targetOffset > .3) {
          turn = .5;
        } else if (targetOffset < -.3) {
          turn = -.5;
        } else if (targetOffset > .1) {
          turn = .3;
        } else if (targetOffset < -.1) {
          turn = -.3;
        }
        // turn = clamp(Math.sqrt(turn), -.9, .9);
        m_robotDrive.arcadeDrive(0.5, turn, false);
      } else {
        m_robotDrive.arcadeDrive(0.4, 0, false);
      }
    } else {
      teleopPeriodic();
    }
  }

  @Override
  public void testExit() {

  }

}
