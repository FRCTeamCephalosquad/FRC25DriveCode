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
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
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

  // Power
  PowerDistribution pdu = new PowerDistribution(1, ModuleType.kRev);

  /**
   * Set up the robot, this is called ONCE when the robot code starts
   */
  public Robot() {
    SendableRegistry.addChild(m_robotDrive, m_leftDrive);
    SendableRegistry.addChild(m_robotDrive, m_rightDrive);

    final double NOMINAL_VOLTAGE = 11.0;

    // Right Motor
    SparkMaxConfig rightConfig = new SparkMaxConfig();
    rightConfig.voltageCompensation(NOMINAL_VOLTAGE);
    m_rightDrive.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Set up right follower
    SparkMaxConfig rightFollowConfig = new SparkMaxConfig();
    rightFollowConfig.follow(m_rightDrive);
    rightFollowConfig.voltageCompensation(NOMINAL_VOLTAGE);
    m_rightDriveFollower.configure(rightFollowConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // left Motor
    SparkMaxConfig leftConfig = new SparkMaxConfig();
    leftConfig.voltageCompensation(NOMINAL_VOLTAGE);
    leftConfig.inverted(true);
    m_leftDrive.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Set up left follower
    SparkMaxConfig leftFollowConfig = new SparkMaxConfig();
    leftFollowConfig.follow(m_leftDrive);
    leftFollowConfig.voltageCompensation(NOMINAL_VOLTAGE);
    m_leftDriveFollower.configure(leftFollowConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    vision.start();

    gyro.resetDisplacement();
    gyro.zeroYaw();
    autonomousOptionSetup();
  }

  /**
   * This is called over and over again while the robot is running.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    // Auto Information
    SmartDashboard.putString("Selected Auto:", auto_chooser.getSelected());

    // Send Gyro Information
    SmartDashboard.putNumber("Gyro", gyro.getYaw());

    // Send power information
    SmartDashboard.putNumber("Voltage", pdu.getVoltage());
    SmartDashboard.putNumber("Total Current", pdu.getTotalCurrent());

    SmartDashboard.putNumber("Drive R1 (Red) Current", pdu.getCurrent(10));
    SmartDashboard.putNumber("Drive R2 (Black) Current", pdu.getCurrent(8));
    SmartDashboard.putNumber("Drive L1 (Blue) Current", pdu.getCurrent(11));
    SmartDashboard.putNumber("Drive L2 (Yellow) Current", pdu.getCurrent(9));

    SmartDashboard.putNumber("Yeeter Motor", pdu.getCurrent(19));
  }

  /**
   * Drive for the given amount of time, at the given speed
   */
  Command driveForTime(double time, double speed) {
    return Commands.sequence(
        new RunCommand(() -> m_robotDrive.arcadeDrive(speed, 0.0, false)).raceWith(new WaitCommand(time)),
        new InstantCommand(() -> m_robotDrive.arcadeDrive(0, 0.0, false)));
  }

  Command driveDistance(double meters, double speed) {
    return new Command() {
      @Override
      public void initialize() {
        gyro.resetDisplacement();
      }

      @Override
      public void execute() {
        m_robotDrive.arcadeDrive(speed, 0.0, false);
      }

      @Override
      public boolean isFinished() {
        return gyro.getDisplacementY() >= meters;
      }

      @Override
      public void end(boolean interrupted) {
        m_robotDrive.arcadeDrive(0.0, 0.0, false);
      }
    };

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
          System.out.println("AprilTag " + tagId + " in sight: " + vision.targetPosition());
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
          System.out.println("AprilTag not in sight. Moving forward.");
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
  private static final String autoRedRight = "Red Right (Red Reef)";
  private static final String autoRedMiddle = "Red Middle";
  private static final String autoRedLeft = "Red Left";

  private static final String autoBlueRight = "Blue Right (Blue Reef)";
  private static final String autoBlueMiddle = "Blue Middle";
  private static final String autoBlueLeft = "Blue Left";

  private final SendableChooser<String> auto_chooser = new SendableChooser<>();

  public void autonomousOptionSetup() {
    auto_chooser.setDefaultOption(autoDefault, autoDefault);
    auto_chooser.addOption(autoRedRight, autoRedRight);
    auto_chooser.addOption(autoRedMiddle, autoRedMiddle);
    auto_chooser.addOption(autoRedLeft, autoRedLeft);
    auto_chooser.addOption(autoBlueRight, autoBlueRight);
    auto_chooser.addOption(autoBlueMiddle, autoBlueMiddle);
    auto_chooser.addOption(autoBlueLeft, autoBlueLeft);
    SmartDashboard.putData("Auto choices", auto_chooser);
  }

  private Command autoLeaveLine() {
    return Commands.sequence(
        driveForTime(2, 0.5),
        park());
  }

  private Command autoRightCommand(int aprilTag) {
    return Commands.sequence(
        driveForTime(2, 0.5),
        turnDegrees(-45),
        seekAprilTagAhead(aprilTag)
            .raceWith(new WaitCommand(7)),
        new WaitCommand(1),
        coralYeeter(), // YEET
        new WaitCommand(1),
        driveForTime(1, -.5),
        park());
  }

  private Command autoMiddleCommand(int aprilTag) {
    return Commands.sequence(
        driveForTime(1, 0.5),
        seekAprilTagAhead(aprilTag)
            .raceWith(new WaitCommand(3)),
            new WaitCommand(1),
        coralYeeter(), // YEET
        new WaitCommand(1),
        driveForTime(1, -.5),
        park());
  }

  private Command autoLeftCommand(int aprilTag) {
    return Commands.sequence(
        driveForTime(2, 0.5),
        turnDegrees(45),
        seekAprilTagAhead(aprilTag)
            .raceWith(new WaitCommand(7)),
        new WaitCommand(1),
        coralYeeter(),
        new WaitCommand(1),
        driveForTime(1, -.5),
        park());
  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {

    final String autoName = auto_chooser.getSelected();

    final Command autoCommand = switch (autoName) {
      case autoRedRight -> autoRightCommand(11);
      case autoRedMiddle -> autoMiddleCommand(10);
      case autoRedLeft -> autoLeftCommand(9);

      case autoBlueRight -> autoRightCommand(20);
      case autoBlueMiddle -> autoMiddleCommand(21);
      case autoBlueLeft -> autoLeftCommand(22);

      default -> autoLeaveLine();
    };

    CommandScheduler.getInstance().schedule(autoCommand);

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
    } else if (m_controller.getBButton()) {
      m_coralFeeder.set(0.3);
    } else {
      // stop
      m_coralFeeder.set(0);
      m_coralFeeder.stopMotor();
    }
  }


  @Override
  public void testInit() {

  }


  @Override
  public void testPeriodic() {

  }

  @Override
  public void testExit() {

  }

}
