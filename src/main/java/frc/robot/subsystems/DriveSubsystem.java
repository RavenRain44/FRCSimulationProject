package frc.robot.subsystems;


import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {

    private Field2d m_field = new Field2d();

    // Subsystem devices
    private Encoder m_leftEncoder = new Encoder(0, 1);
    private Encoder m_rightEncoder = new Encoder(2, 3);

    private PWMSparkMax m_leftMotor = new PWMSparkMax(0);
    private PWMSparkMax m_rightMotor = new PWMSparkMax(1);

    private AnalogGyro m_gyro = new AnalogGyro(1);

    // Odometry class for tracking robot pose
    private DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d(),
            m_leftEncoder.getDistance(), m_rightEncoder.getDistance(),
            new Pose2d(5.0, 13.5, new Rotation2d()));

    // Simulation classes
    private EncoderSim m_leftEncoderSim = new EncoderSim(m_leftEncoder);
    private EncoderSim m_rightEncoderSim = new EncoderSim(m_rightEncoder);

    private AnalogGyroSim m_gyroSim = new AnalogGyroSim(m_gyro);

    DifferentialDrivetrainSim driveSim = new DifferentialDrivetrainSim(
            Constants.DriveConstants.kDriveMotors,
            Constants.DriveConstants.kDriveGearReduction,
            Constants.DriveConstants.kDriveMOI,
            Constants.DriveConstants.kDriveMass,
            Constants.DriveConstants.kDriveWheelRadius,
            Constants.DriveConstants.kTrackwidthMeters,
            VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005)
    );

    public DriveSubsystem() {
        m_leftEncoder.setDistancePerPulse(2 * Math.PI * Constants.DriveConstants.kDriveWheelRadius / 2048.0);
        m_rightEncoder.setDistancePerPulse(2 * Math.PI * Constants.DriveConstants.kDriveWheelRadius / 2048.0);

        SmartDashboard.putData("Field", m_field);
    }

    public void periodic() {
        m_odometry.update(
                m_gyro.getRotation2d(),
                m_leftEncoder.getDistance(),
                m_rightEncoder.getDistance());
        m_field.setRobotPose(m_odometry.getPoseMeters());
    }

    public void simulationPeriodic() {
        driveSim.setInputs(m_leftMotor.get() * RobotController.getInputVoltage(),
                            m_rightMotor.get() * RobotController.getInputVoltage());
        driveSim.update(0.02);

        m_leftEncoderSim.setDistance(driveSim.getLeftPositionMeters());
        m_leftEncoderSim.setRate(driveSim.getLeftVelocityMetersPerSecond());
        m_rightEncoderSim.setDistance(driveSim.getRightPositionMeters());
        m_rightEncoderSim.setRate(driveSim.getRightVelocityMetersPerSecond());
        m_gyroSim.setAngle(-driveSim.getHeading().getDegrees());
    }

    public void drive(double leftSpeed, double rightSpeed) {
        m_leftMotor.set(leftSpeed);
        m_rightMotor.set(rightSpeed);

    }
}

