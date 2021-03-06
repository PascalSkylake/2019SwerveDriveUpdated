package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends SubsystemBase {
    private final AnalogInput ai;

    private double p, i, d;
    // private static final int kEncoderResolution = 2048;

    private static final double MIN_VOLTAGE = 0.2, MAX_VOLTAGE = 4.76, DELTA_VOLTAGE = MAX_VOLTAGE - MIN_VOLTAGE;
    // distancePerPulse = (0.0254 * 4 * Math.PI * 12 * 19) / (kEncoderResolution *
    // 32 * 60);
    // kModuleMaxAngularVelocity = SwerveDrive.kMaxAngularSpeed,
    // kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared;

    private String name;
    private double offset;
    private boolean isInverted;

    private final WPI_TalonSRX m_driveMotor;
    private final WPI_TalonSRX m_turningMotor;

    private final Encoder m_driveEncoder;

    private final PIDController m_turningPIDController;

    // private final PIDController m_drivePIDController = new PIDController(1 /
    // 10.0, 0, 0);

    @Override
    public void periodic() {
    }

    /**
     * Constructs a SwerveModule.
     *
     * @param driveMotorChannel   ID for the drive motor.
     * @param turningMotorChannel ID for the turning motor.
     * @param aiPort              Port num of the Analog Input
     * @param turnP               P Val of Turn PID
     * @param turnI               I Val of Turn PID
     * @param TurnD               D Val of Turn PID
     * @param encA                Port of Encoder A Channel
     * @param encB                Port of Encoder B Channel
     * @param offset              Offset of the Pivot Motor, 0 by default
     */
    public SwerveModule(String name, int driveMotorChannel, int turningMotorChannel, int aiPort, double turnP, double turnI,
                        double turnD, int encA, int encB, boolean reversed) {
        this.name = name;
        isInverted = reversed;

        ai = new AnalogInput(aiPort);
        m_driveMotor = new WPI_TalonSRX(driveMotorChannel);
        m_turningMotor = new WPI_TalonSRX(turningMotorChannel);

        p = turnP;
        i = turnI;
        d = turnD;

        m_turningPIDController = new PIDController(p, i, d);
        m_driveEncoder = new Encoder(encA, encB);

        // Set the distance per pulse for the drive encoder. We can simply use the
        // distance traveled for one rotation of the wheel divided by the encoder
        // resolution.
        // m_driveEncoder.setDistancePerPulse(distancePerPulse);

        // // Set the distance (in this case, angle) per pulse for the turning encoder.
        // // This is the the angle through an entire rotation (2 * wpi::math::pi)
        // // divided by the encoder resolution.
        // ai.setDistancePerPulse(2 * Math.PI / kEncoderResolution);

        // Limit the PID Controller's input range between -pi and pi and set the input
        // to be continuous.
        m_turningPIDController.enableContinuousInput(MIN_VOLTAGE, MAX_VOLTAGE);

        var tab = Shuffleboard.getTab(name + " Module");
        tab.addNumber("Angle", this::getAngle);
        tab.addNumber("Speed", () -> (isInverted ? -1 : 1) * getState().speedMetersPerSecond);
        tab.addNumber("Target Angle", m_turningPIDController::getSetpoint);
        tab.addNumber("Target Error", m_turningPIDController::getPositionError);
        tab.addBoolean("At Setpoint", m_turningPIDController::atSetpoint);
        tab.addBoolean("Is Flipped", () -> this.isInverted);
    }

    public SwerveModule(String name, int driveMotorChannel, int turningMotorChannel, int aiPort, double pVal, double iVal, double dVal,
                        int encA, int encB) {
        this(name, driveMotorChannel, turningMotorChannel, aiPort, pVal, iVal, dVal, encA, encB, false);
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(m_driveEncoder.getRate(), new Rotation2d(Math.toRadians(getAngle())));
    }

    /**
     * Sets the desired state for the module.
     *
     * @param state Desired state with speed and angle.
     */
    public double setDesiredState(SwerveModuleState state) {
        // Calculate the drive output from the drive PID controller.
        // final var driveOutput =
        // m_drivePIDController.calculate(m_driveEncoder.getRate(),
        // state.speedMetersPerSecond);

        // Calculate the turning motor output from the turning PID controller.
        double targetAngle = to360(state.angle.getDegrees());
        double dAngle = Math.abs(toAngle(ai.getAverageVoltage()) - targetAngle);

        boolean isFlipped = dAngle >= 90 && dAngle <= 270;


        final var turnOutput = m_turningPIDController.calculate(ai.getAverageVoltage(),
            degreesToVolts((targetAngle + (isFlipped ? 180.0 : 0)) % 360.0));

        // Calculate the turning motor output from the turning PID controller.
        double driveCalculation = state.speedMetersPerSecond;

        if (isFlipped) {
            driveCalculation = -driveCalculation;
        }
        if (isInverted) {
            driveCalculation = -driveCalculation;
        }

        m_driveMotor.set(driveCalculation);
        m_turningMotor.set(turnOutput);

        return state.angle.getDegrees();
    }

    public double toAngle(double voltage) {
        return ((360.0 * (voltage - MIN_VOLTAGE) / DELTA_VOLTAGE) + 360.0 - offset) % 360;
    }

    public double voltsToRadians(double voltage) {
        return Math.toRadians(toAngle(voltage));
    }

    public double degreesToVolts(double angle) {
        return ((DELTA_VOLTAGE / 360.0) * angle) + MIN_VOLTAGE;
    }

    public double getAngle() {
        var angle = toAngle(ai.getAverageVoltage());
        if (angle > 180) {
            angle -= 360;
        }

        return -angle;
    }

    private double to360(double angle) {
        if (angle < 0) {
            angle = -angle;
        } else if (angle > 0) {
            angle = -angle;
            angle += 360;
        }
        return angle;
    }
}
