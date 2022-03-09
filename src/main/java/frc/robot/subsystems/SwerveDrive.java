package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;

public class SwerveDrive extends SubsystemBase {
    public static final double kMaxSpeed = 1.0; // 1 meters per second
    public static final double kMaxAngularSpeed = 1; // 1 radian per second

    private final Translation2d m_frontLeftLocation = new Translation2d(0.2667, 0.327025);
    private final Translation2d m_frontRightLocation = new Translation2d(0.2667, -0.327025);
    private final Translation2d m_backLeftLocation = new Translation2d(-0.2667, 0.327025);
    private final Translation2d m_backRightLocation = new Translation2d(-0.2667, -0.327025);

    private PIDController targetPid;

    private final SwerveModule m_frontLeft = new SwerveModule(
        "FL",
        RobotMap.FRONT_LEFT_DRIVE,
        RobotMap.FRONT_LEFT_PIVOT,
        RobotMap.FRONT_LEFT_ANALOG,
        Constants.FRONT_LEFT_P,
        Constants.FRONT_LEFT_I,
        Constants.FRONT_LEFT_D,
        RobotMap.FRONT_LEFT_DRIVE_ENC_A,
        RobotMap.FRONT_LEFT_DRIVE_ENC_B
    );
    private final SwerveModule m_frontRight = new SwerveModule(
        "FR",
        RobotMap.FRONT_RIGHT_DRIVE,
        RobotMap.FRONT_RIGHT_PIVOT,
        RobotMap.FRONT_RIGHT_ANALOG,
        Constants.FRONT_RIGHT_P,
        Constants.FRONT_RIGHT_I,
        Constants.FRONT_RIGHT_D,
        RobotMap.FRONT_RIGHT_DRIVE_ENC_A,
        RobotMap.FRONT_RIGHT_DRIVE_ENC_B,
        true
    );
    private final SwerveModule m_backLeft = new SwerveModule(
        "RL",
        RobotMap.REAR_LEFT_DRIVE,
        RobotMap.REAR_LEFT_PIVOT,
        RobotMap.REAR_LEFT_ANALOG,
        Constants.REAR_LEFT_P,
        Constants.REAR_LEFT_I,
        Constants.REAR_LEFT_D,
        RobotMap.REAR_LEFT_DRIVE_ENC_A,
        RobotMap.REAR_LEFT_DRIVE_ENC_B
    );
    private final SwerveModule m_backRight = new SwerveModule(
        "RR",
        RobotMap.REAR_RIGHT_DRIVE,
        RobotMap.REAR_RIGHT_PIVOT,
        RobotMap.REAR_RIGHT_ANALOG,
        Constants.REAR_RIGHT_P,
        Constants.REAR_RIGHT_I,
        Constants.REAR_RIGHT_D,
        RobotMap.REAR_RIGHT_DRIVE_ENC_A,
        RobotMap.REAR_RIGHT_DRIVE_ENC_B,
        true
    );

    // private final AnalogGyro m_gyro = new AnalogGyro(0);

    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_frontLeftLocation,
        m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

    private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, this.getAngle());

    public SwerveDrive() {
        RobotContainer.navx.reset();
        var tab = Shuffleboard.getTab("Swerve Drive");
        tab.addNumber("Angle", () -> -RobotContainer.navx.getYaw());

        targetPid = new PIDController(Constants.TARGET_P, Constants.TARGET_I, Constants.TARGET_D);
        targetPid.enableContinuousInput(-180.0, 180.0);
        targetPid.setTolerance(2);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Navx Angle", getAngle().getDegrees());
    }

    /**
     * Returns the angle of the robot as a Rotation2d.
     *
     * @return The angle of the robot.
     */
    public Rotation2d getAngle() {
        // Negating the angle because WPILib gyros are CW positive.
        return Rotation2d.fromDegrees(-RobotContainer.navx.getAngle());
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     */
    @SuppressWarnings("ParameterName")
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        var swerveModuleStates = m_kinematics
            .toSwerveModuleStates(fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getAngle())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_backLeft.setDesiredState(swerveModuleStates[2]);
        m_backRight.setDesiredState(swerveModuleStates[3]);

        // System.out.println("FL Angle: " + swerveModuleStates[0].angle.getDegrees());
        // System.out.println("FR Angle: " + swerveModuleStates[1].angle.getDegrees());
        // System.out.println("RL Angle: " + swerveModuleStates[2].angle.getDegrees());
        // System.out.println("RR Angle: " + swerveModuleStates[3].angle.getDegrees());
    }

    /**
     * Updates the field relative position of the robot.
     */
    public void updateOdometry() {
        m_odometry.update(this.getAngle(), m_frontLeft.getState(), m_frontRight.getState(), m_backLeft.getState(),
            m_backRight.getState());
    }

    public void setPID(double p, double i, double d) {
        this.targetPid.setPID(p, i, d);
    }

    public void rotateToAngleInPlace(double setAngle) {
        holdAngleWhileDriving(0, 0, setAngle, false);
    }

    public void holdAngleWhileDriving(double x, double y, double setAngle, boolean fod) {
        var rotateOutput = targetPid.calculate(-RobotContainer.navx.getYaw(), normalizeAngle(setAngle)) * kMaxAngularSpeed;
        this.drive(x, y, MathUtil.clamp(rotateOutput, -1, 1), fod);
    }

    public void stop() {
        RobotContainer.swerveDrive.drive(0,0,0,false);
        targetPid.reset();
    }

    public void resetNavx() {
        targetPid.reset();
        RobotContainer.navx.reset();
    }

    public void resetPid() {
        targetPid.reset();
    }

    private static double normalizeAngle(double angle) {
        if(angle > 0) {
            angle %= 360;
            if(angle > 180) {
                angle -= 360;
            }
        }
        else if(angle < 0) {
            angle %= -360;
            if(angle < -180) {
                angle += 360;
            }
        }
        return angle;
    }
}