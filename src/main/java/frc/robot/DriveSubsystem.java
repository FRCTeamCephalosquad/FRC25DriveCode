package frc.robot;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
    // Drive Motors
    private final SparkMax m_leftDrive = new SparkMax(2, MotorType.kBrushed);
    private final SparkMax m_rightDrive = new SparkMax(3, MotorType.kBrushed);
    private final SparkMax m_leftDriveFollower = new SparkMax(4, MotorType.kBrushed);
    private final SparkMax m_rightDriveFollower = new SparkMax(5, MotorType.kBrushed);

    // Drive Control
    private final DifferentialDrive m_robotDrive;

    // Encoders

    private final Encoder leftEncoder = new Encoder(0, 1);
    private final Encoder rightEncoder = new Encoder(2, 3);

    private final PIDController rightPID = new PIDController(0.9, 0.02, 0.05);
    private final PIDController leftPID = new PIDController(0.9, 0.02, 0.05);

    void drivePeriodic() {
       // System.out.printf("R: %.2f / %.2f L: %.2f / %.2f\n", rightEncoder.getRate(), m_rightDrive.get(), leftEncoder.getRate(), m_leftDrive.get());
    }

    public DriveSubsystem() {

        final double NOMINAL_VOLTAGE = 11.0;

        // Right Motor
        SparkMaxConfig rightConfig = new SparkMaxConfig();
        rightConfig.voltageCompensation(NOMINAL_VOLTAGE);
        m_rightDrive.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Set up right follower
        SparkMaxConfig rightFollowConfig = new SparkMaxConfig();
        rightFollowConfig.follow(m_rightDrive);
        rightFollowConfig.voltageCompensation(NOMINAL_VOLTAGE);
        m_rightDriveFollower.configure(rightFollowConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

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

        {
            // Encoder Setup
            final double wheelRadiusM = Inches.of(6).in(Meters);
            final double wheelCircumfrenceM = wheelRadiusM * Math.PI;
            final int ppr = 128;
            final double metersPerPulse = wheelCircumfrenceM / ppr;
            leftEncoder.setDistancePerPulse(metersPerPulse);
            rightEncoder.setDistancePerPulse(metersPerPulse);
        }

        rightPID.setIntegratorRange(-0.5, 0.5);
        leftPID.setIntegratorRange(-0.5, 0.5);

        m_robotDrive = new DifferentialDrive(m_leftDrive::set, m_rightDrive::set);
    }

    public void resetDistanceForward(){
        rightEncoder.reset();
        leftEncoder.reset();
    }

    public double getDistanceForward(){
        return ( rightEncoder.getDistance() + leftEncoder.getDistance() ) / 2;
    }

    public void setMaxOutput(double m) {
        m_robotDrive.setMaxOutput(m);
    }

    public void stopMotor() {
        m_robotDrive.stopMotor();
    }

    public void arcadeDriveJoystick(double xSpeed, double zRotation) {
        m_robotDrive.setDeadband(RobotDriveBase.kDefaultDeadband);
        m_robotDrive.arcadeDrive(xSpeed, zRotation, true);
        m_robotDrive.setDeadband(0);
    }

    public void arcadeDrive(double xSpeed, double zRotation) {
        m_robotDrive.setDeadband(0);
        m_robotDrive.arcadeDrive(xSpeed, zRotation, false);
        m_robotDrive.setDeadband(RobotDriveBase.kDefaultDeadband);
    }
}
