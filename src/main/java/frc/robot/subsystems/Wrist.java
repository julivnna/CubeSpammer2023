package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;
import frc.robot.filters.ExponentialSmoothingFilter;
import frc.robot.util.NerdyMath;

public class Wrist extends SubsystemBase implements Reportable {
    private TalonFX leftWrist;
    private TalonFX rightWrist;
    private int targetTicks = WristConstants.kWristStow;
    public BooleanSupplier atTargetPosition;
    private TalonSRX leftEncoder;
    private ExponentialSmoothingFilter joystickFilter = new ExponentialSmoothingFilter(WristConstants.kLowPassAlpha);

    public Wrist() {
        leftWrist = new TalonFX(WristConstants.kLeftWristID);
        rightWrist = new TalonFX(WristConstants.kRightWristID);
        leftEncoder = new TalonSRX(WristConstants.kLeftEncoderID);
    }

    public void init() {
        leftWrist.setNeutralMode(NeutralMode.Brake);
        leftWrist.setInverted(false);
        rightWrist.follow(leftWrist);
        rightWrist.setNeutralMode(NeutralMode.Brake);
        rightWrist.setInverted(TalonFXInvertType.OpposeMaster);
        leftEncoder.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.QuadEncoder, 0, 1000);
        leftEncoder.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.PulseWidthEncodedPosition, 1, 1000);
        //test
        leftWrist.config_kP(0, WristConstants.kWristP);
        leftWrist.config_kI(0, WristConstants.kWristI);
        leftWrist.config_kD(0, WristConstants.kWristD);
        leftWrist.config_kF(0, WristConstants.kWristF);

        leftWrist.configMotionCruiseVelocity(WristConstants.kWristCruiseVelocity);
        leftWrist.configMotionAcceleration(WristConstants.kWristMotionAcceleration);

    }

    public void resetEncoders(){
        double absoluteTicks = leftEncoder.getSelectedSensorPosition(0);
        leftWrist.setSelectedSensorPosition(absoluteTicks * WristConstants.kFalconTicksPerAbsoluteTicks, 0, 100);
    }

    public void moveWristJoystick(double currentJoystickOutput) {
        if (currentJoystickOutput > WristConstants.kWristDeadband) {
            if (leftWrist.getStatorCurrent() >= 45)
            {
                leftWrist.set(ControlMode.PercentOutput, 0);
            } else {
                leftWrist.set(ControlMode.PercentOutput, 0.3);
            }
        }

        else if (currentJoystickOutput < -WristConstants.kWristDeadband){
            if (leftWrist.getStatorCurrent() >= 45) {
                leftWrist.set(ControlMode.PercentOutput, 0);
            }
            else {
                leftWrist.set(ControlMode.PercentOutput, -0.3);
            }}

        else {
            leftWrist.set(ControlMode.PercentOutput, 0);
            leftWrist.setNeutralMode(NeutralMode.Brake);
        }
    }

    public void moveArmMotionMagicJoystick(double joystickInput, double percentExtended) {
        if (joystickInput < -0.1 || joystickInput > 0.1) {
            int tickChange = (int) (WristConstants.kJoystickScale * joystickInput);
            int currentTicks = (int) leftWrist.getSelectedSensorPosition();

            tickChange = (int) joystickFilter.calculate(tickChange);

            targetTicks = currentTicks + tickChange;
            targetTicks = (int) NerdyMath.clamp(targetTicks, WristConstants.kWristLowerLimit, WristConstants.kWristUpperLimit);
        }
        else {
            joystickFilter.calculate(0);
        }

        setTargetTicks(targetTicks);
    }


    public double getWristAngle() {
        double ticks = leftWrist.getSelectedSensorPosition(0);
        double angle = ticks * WristConstants.kDegreesPerTick %360;
        return angle;
    }

    public double getWristAngleRadians() {
        return Math.toRadians(getWristAngle());
    }

    public void moveWristMotionMagic() {
        double ff = WristConstants.kWristFF * Math.cos(getWristAngleRadians());
        leftWrist.set(ControlMode.MotionMagic, targetTicks, DemandType.ArbitraryFeedForward, ff);
    }

    public void moveWristMotionMagicButton(int position) {
        leftWrist.configMotionCruiseVelocity(WristConstants.kWristCruiseVelocity);
        leftWrist.configMotionAcceleration(WristConstants.kWristMotionAcceleration);
        setTargetTicks(position);
    }

    public void setTargetTicks(int targetTicks) {
        this.targetTicks = targetTicks;
    }

    @Override
    public void reportToSmartDashboard(LOG_LEVEL priority) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void initShuffleboard(LOG_LEVEL priority) {
        // TODO Auto-generated method stub
        
    }
    
}
