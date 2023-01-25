// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.kMotor;
import frc.robot.Constants.kCANCoder;
import frc.robot.Constants.kGyro;

import java.util.ArrayList;


/**
 * The VM is configured to automatically run this class, and to call the methods corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private enum TestType {
        QUASISTATIC,
        DYNAMIC,
        OTHER
    }

    // private final CANSparkMax leftLeader = new CANSparkMax(Constants.LEFT_TOP_PORT, MotorType.kBrushless);
    // private final CANSparkMax rightLeader = new CANSparkMax(Constants.RIGHT_TOP_PORT, MotorType.kBrushless);
    // private final RelativeEncoder leftEncoder = leftLeader.getEncoder();
    // private final RelativeEncoder rightEncoder = rightLeader.getEncoder();
    // private final AHRS gyro = new AHRS();

    private final CANSparkMax mot_leftFrontDrive = new CANSparkMax(kMotor.id_leftFrontDrive, MotorType.kBrushless);
    private final CANSparkMax mot_leftRearDrive = new CANSparkMax(kMotor.id_leftRearDrive, MotorType.kBrushless);
    private final CANSparkMax mot_rightFrontDrive = new CANSparkMax(kMotor.id_rightFrontDrive, MotorType.kBrushless);
    private final CANSparkMax mot_rightRearDrive = new CANSparkMax(kMotor.id_rightRearDrive, MotorType.kBrushless);

    private final WPI_CANCoder enc_leftDrive = new WPI_CANCoder(kCANCoder.id_leftEncoder);
    private final WPI_CANCoder enc_rightDrive = new WPI_CANCoder(kCANCoder.id_rightEncoder);

    private final CANCoderConfiguration enc_config = new CANCoderConfiguration();

    private final WPI_PigeonIMU m_gyro = new WPI_PigeonIMU(kGyro.id_gyro);

    private final ArrayList<Double> data = new ArrayList<>(Constants.DATA_VECTOR_SIZE);
    private double voltage = 0.0;
    private double leftVoltage = 0.0;
    private double rightVoltage = 0.0;
    private double currentTime = 0.0;

    private boolean rotate = false;
    private TestType testType = TestType.OTHER;
    private double voltageCommand = 0.0;
    private double startTime = 0.0;

    public Robot() {
        super(0.005);

        mot_leftFrontDrive.restoreFactoryDefaults();
        mot_leftRearDrive.restoreFactoryDefaults();
        mot_rightFrontDrive.restoreFactoryDefaults();
        mot_rightRearDrive.restoreFactoryDefaults();

        mot_leftFrontDrive.setInverted(kMotor.leftInverted);
        mot_leftRearDrive.setInverted(kMotor.leftInverted);
        mot_rightFrontDrive.setInverted(kMotor.rightInverted);
        mot_rightRearDrive.setInverted(kMotor.rightInverted);

        mot_leftRearDrive.follow(mot_leftFrontDrive);
        mot_rightRearDrive.follow(mot_rightFrontDrive);

        mot_leftFrontDrive.setIdleMode(IdleMode.kBrake);
        mot_leftRearDrive.setIdleMode(IdleMode.kBrake);
        mot_rightFrontDrive.setIdleMode(IdleMode.kBrake);
        mot_rightRearDrive.setIdleMode(IdleMode.kBrake);

        enc_config.sensorCoefficient = kCANCoder.enc_SensorCoefficient;
        enc_config.unitString = kCANCoder.enc_UnitString;
        enc_config.sensorTimeBase = SensorTimeBase.PerSecond;
        enc_leftDrive.configAllSettings(enc_config);
        enc_rightDrive.configAllSettings(enc_config);

        LiveWindow.disableAllTelemetry();
    }

    /**
     * This method is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        SmartDashboard.putString("SysIdTestType", "Quasistatic");
        SmartDashboard.putNumber("SysIdVoltageCommand", 0);
    }
    
    
    /**
     * This method is called every robot packet, no matter the mode. Use this for items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic methods, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {}
    
    
    /**
     * This autonomous (along with the chooser code above) shows how to select between different
     * autonomous modes using the dashboard. The sendable chooser code works with the Java
     * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
     * uncomment the getString line to get the auto name from the text box below the Gyro
     *
     * <p>You can add additional auto modes by adding additional comparisons to the switch structure
     * below with additional strings. If using the SendableChooser make sure to add them to the
     * chooser code above as well.
     */
    @Override
    public void autonomousInit() {
        // Initialize logging
        String testTypeString = SmartDashboard.getString("SysIdTestType", "");
        if (testTypeString.equals("Quasistatic")) {
            testType = TestType.QUASISTATIC;
        } else if (testTypeString.equals("Dynamic")) {
            testType = TestType.DYNAMIC;
        } else {
            testType = TestType.OTHER;
        }

        rotate = SmartDashboard.getBoolean("SysIdRotate", false);
        voltageCommand = SmartDashboard.getNumber("SysIdVoltageCommand", 0.0);
        startTime = Timer.getFPGATimestamp();
        data.clear();
    }
    
    
    /** This method is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
        logData(-enc_leftDrive.getPosition(), enc_rightDrive.getPosition(), -enc_leftDrive.getVelocity(), enc_rightDrive.getVelocity(), Math.toRadians(m_gyro.getAngle()), Math.toRadians(m_gyro.getRate()));

        mot_leftFrontDrive.setVoltage(leftVoltage);
        mot_rightFrontDrive.setVoltage(rightVoltage);
    }


    /** This method is called once when teleop is enabled. */
    @Override
    public void teleopInit() {}
    
    
    /** This method is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        pushNTDiagnostics();
    }
    
    
    /** This method is called once when the robot is disabled. */
    @Override
    public void disabledInit() {
        mot_leftFrontDrive.setVoltage(0.0);
        mot_rightFrontDrive.setVoltage(0.0);
        sendData();
    }


    /** This method is called periodically when disabled. */
    @Override
    public void disabledPeriodic() {
        pushNTDiagnostics();
    }
    
    
    /** This method is called once when test mode is enabled. */
    @Override
    public void testInit() {}
    
    
    /** This method is called periodically during test mode. */
    @Override
    public void testPeriodic() {
        pushNTDiagnostics();
    }

    private void updateData() {
        currentTime = Timer.getFPGATimestamp();
        if (testType == TestType.QUASISTATIC) {
            voltage = voltageCommand * (currentTime - startTime);
        } else if (testType == TestType.DYNAMIC) {
            voltage = voltageCommand;
        } else {
            voltage = 0.0;
        }
    }

    private void logData(double leftPosition, double rightPosition, double leftRate, double rightRate, double gyroPosition, double gyroRate) {
        updateData();
        if (data.size() < Constants.DATA_VECTOR_SIZE) {
            data.add(currentTime);
            data.add(leftVoltage);
            data.add(rightVoltage);
            data.add(leftPosition);
            data.add(rightPosition);
            data.add(leftRate);
            data.add(rightRate);
            data.add(gyroPosition);
            data.add(gyroRate);
        }
        leftVoltage = voltage * (rotate ? -1 : 1);
        rightVoltage = voltage;
    }

    private void sendData() {
        SmartDashboard.putBoolean("SysIdOverflow", data.size() > Constants.DATA_VECTOR_SIZE);

        String dataString = data.toString();
        SmartDashboard.putString("SysIdTelemetry", data.toString().substring(1, dataString.length() - 1));

        reset();
    }

    private void reset() {
        voltage = 0.0;
        currentTime = 0.0;
        startTime = 0.0;
        leftVoltage = 0.0;
        rightVoltage = 0.0;
        data.clear();
    }

    private void pushNTDiagnostics() {
        SmartDashboard.putNumber("Left Position", -enc_leftDrive.getPosition());
        SmartDashboard.putNumber("Right Position", enc_rightDrive.getPosition());
        SmartDashboard.putNumber("Left Velocity", -enc_leftDrive.getVelocity());
        SmartDashboard.putNumber("Right Velocity", enc_rightDrive.getVelocity());
        SmartDashboard.putNumber("Gyro Reading", Math.toRadians(m_gyro.getAngle()));
        SmartDashboard.putNumber("Gyro Rate", Math.toRadians(m_gyro.getRate()));
    }
}
