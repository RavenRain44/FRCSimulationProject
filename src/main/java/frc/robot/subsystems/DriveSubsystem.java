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

    private final Field2d field = new Field2d();

    // Subsystem devices
    private final Encoder leftEncoder = new Encoder(0, 1);
    private final Encoder rightEncoder = new Encoder(2, 3);

    private final PWMSparkMax leftMotor = new PWMSparkMax(0);
    private final PWMSparkMax rightMotor = new PWMSparkMax(1);

    private final AnalogGyro gyro = new AnalogGyro(1);

    // Odometry class for tracking robot pose
    private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(gyro.getRotation2d(),
            leftEncoder.getDistance(), rightEncoder.getDistance(),
            new Pose2d(5.0, 13.5, new Rotation2d()));

    // Simulation classes
    private final EncoderSim leftEncoderSim = new EncoderSim(leftEncoder);
    private final EncoderSim rightEncoderSim = new EncoderSim(rightEncoder);

    private final AnalogGyroSim gyroSim = new AnalogGyroSim(gyro);

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
        leftEncoder.setDistancePerPulse(2 * Math.PI * Constants.DriveConstants.kDriveWheelRadius / 2048.0);
        rightEncoder.setDistancePerPulse(2 * Math.PI * Constants.DriveConstants.kDriveWheelRadius / 2048.0);

        SmartDashboard.putData("Field", field);
    }

    public void periodic() {
        odometry.update(
                gyro.getRotation2d(),
                leftEncoder.getDistance(),
                rightEncoder.getDistance());
        field.setRobotPose(odometry.getPoseMeters());
    }

    public void simulationPeriodic() {
        driveSim.setInputs(leftMotor.get() * RobotController.getInputVoltage(),
                            rightMotor.get() * RobotController.getInputVoltage());
        driveSim.update(0.02);

        leftEncoderSim.setDistance(driveSim.getLeftPositionMeters());
        leftEncoderSim.setRate(driveSim.getLeftVelocityMetersPerSecond());
        rightEncoderSim.setDistance(driveSim.getRightPositionMeters());
        rightEncoderSim.setRate(driveSim.getRightVelocityMetersPerSecond());
        gyroSim.setAngle(-driveSim.getHeading().getDegrees());

        field.setRobotPose(driveSim.getPose());
    }

    public void drive(double forwardSpeed, double turnSpeed) {
        leftMotor.set(-forwardSpeed + turnSpeed);
        rightMotor.set(-forwardSpeed - turnSpeed);

    }
}

