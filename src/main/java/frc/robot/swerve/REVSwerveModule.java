package frc.robot.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.Constants.ModuleConstants;
import frc.robot.swerve.SwerveUtils.*;



public class REVSwerveModule extends SwerveModule{
    private final CANSparkMax m_drivingSparkMax;
    private final CANSparkMax m_turningSparkMax;
  
    private final RelativeEncoder m_drivingEncoder;
    private final AbsoluteEncoder m_turningEncoder;
  
    private final SparkMaxPIDController m_drivingPIDController;
    private final SparkMaxPIDController m_turningPIDController;

    private double m_chassisAngularOffset = 0;
    private SwerveModuleStates m_desiredState = new SwerveModuleStates();

    public REVSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset, Translation2d module_location) {
        super(module_location);

        m_drivingSparkMax = new CANSparkMax(drivingCANId, MotorType.kBrushless);
        m_turningSparkMax = new CANSparkMax(turningCANId, MotorType.kBrushless);
    
        // Factory reset, so we get the SPARKS MAX to a known state before configuring
        // them. This is useful in case a SPARK MAX is swapped out.
        m_drivingSparkMax.restoreFactoryDefaults();
        m_turningSparkMax.restoreFactoryDefaults();
    
        // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
        m_drivingEncoder = m_drivingSparkMax.getEncoder();
        m_turningEncoder = m_turningSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
        m_drivingPIDController = m_drivingSparkMax.getPIDController();
        m_turningPIDController = m_turningSparkMax.getPIDController();
        m_drivingPIDController.setFeedbackDevice(m_drivingEncoder);
        m_turningPIDController.setFeedbackDevice(m_turningEncoder);
    
        // Apply position and velocity conversion factors for the driving encoder. The
        // native units for position and velocity are rotations and RPM, respectively,
        // but we want meters and meters per second to use with WPILib's swerve APIs.
        m_drivingEncoder.setPositionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor);
        m_drivingEncoder.setVelocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);
    
        // Apply position and velocity conversion factors for the turning encoder. We
        // want these in radians and radians per second to use with WPILib's swerve
        // APIs.
        m_turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor);
        m_turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);
    
        // Invert the turning encoder, since the output shaft rotates in the opposite direction of
        // the steering motor in the MAXSwerve Module.
        m_turningEncoder.setInverted(ModuleConstants.kTurningEncoderInverted);
    
        // Enable PID wrap around for the turning motor. This will allow the PID
        // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
        // to 10 degrees will go through 0 rather than the other direction which is a
        // longer route.
        m_turningPIDController.setPositionPIDWrappingEnabled(true);
        m_turningPIDController.setPositionPIDWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput);
        m_turningPIDController.setPositionPIDWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput);
    
        // Set the PID gains for the driving motor. Note these are example gains, and you
        // may need to tune them for your own robot!
        m_drivingPIDController.setP(ModuleConstants.kDrivingP);
        m_drivingPIDController.setI(ModuleConstants.kDrivingI);
        m_drivingPIDController.setD(ModuleConstants.kDrivingD);
        m_drivingPIDController.setFF(ModuleConstants.kDrivingFF);
        m_drivingPIDController.setOutputRange(ModuleConstants.kDrivingMinOutput,
            ModuleConstants.kDrivingMaxOutput);
    
        // Set the PID gains for the turning motor. Note these are example gains, and you
        // may need to tune them for your own robot!
        m_turningPIDController.setP(ModuleConstants.kTurningP);
        m_turningPIDController.setI(ModuleConstants.kTurningI);
        m_turningPIDController.setD(ModuleConstants.kTurningD);
        m_turningPIDController.setFF(ModuleConstants.kTurningFF);
        m_turningPIDController.setOutputRange(ModuleConstants.kTurningMinOutput,
            ModuleConstants.kTurningMaxOutput);
    
        m_drivingSparkMax.setIdleMode(ModuleConstants.kDrivingMotorIdleMode);
        m_turningSparkMax.setIdleMode(ModuleConstants.kTurningMotorIdleMode);
        m_drivingSparkMax.setSmartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
        m_turningSparkMax.setSmartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);
    
        // Save the SPARK MAX configurations. If a SPARK MAX browns out during
        // operation, it will maintain the above configurations.
        m_drivingSparkMax.burnFlash();
        m_turningSparkMax.burnFlash();
    
        m_chassisAngularOffset = chassisAngularOffset;
        m_desiredState.rotation = m_turningEncoder.getPosition();
        m_drivingEncoder.setPosition(0);
      }
    
    

    @Override
    public void setState(double linear_vel, double steer_angle_rad, double linear_acc, double steer_angular_vel) {
        m_desiredState.linear_velocity = linear_vel;
        m_desiredState.rotation = steer_angle_rad + m_chassisAngularOffset;
        m_desiredState.linear_acceleration = linear_acc;
        m_desiredState.angular_velocity = steer_angular_vel;

        // Command driving and turning SPARKS MAX towards their respective setpoints.
        m_drivingPIDController.setReference(m_desiredState.linear_velocity, CANSparkMax.ControlType.kVelocity);
        m_turningPIDController.setReference(m_desiredState.rotation, CANSparkMax.ControlType.kPosition);
    }

    @Override
    public void stop() {
        // TODO Auto-generated method stub
        // this - set the motors to stop
        
    }

    @Override
    public double getSteeringAngle() {
        // TODO Auto-generated method stub
        return (m_turningEncoder.getPosition() - m_chassisAngularOffset);
    }

    @Override
    public double getWheelDisplacement() {
        // TODO Auto-generated method stub
        return m_drivingEncoder.getPosition();
    }

    @Override
    public double getWheelVelocity(){
        return m_drivingEncoder.getVelocity();
    }

    @Override
    public double getSteeringRate(){
        return m_turningEncoder.getVelocity();
    }

    @Override
	public void initSendable(SendableBuilder b){
        super.initSendable(b);
        b.addDoubleProperty("Module Steerrate (Rad per s)", this::getSteeringRate, null);
		b.addDoubleProperty("Module Displacement (M per s)", this::getWheelVelocity, null);
    }
    
}
