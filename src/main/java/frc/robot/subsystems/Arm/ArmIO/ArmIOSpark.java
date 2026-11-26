package frc.robot.subsystems.Arm.ArmIO;


import frc.robot.subsystems.Arm.ArmConstants;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ArmIOSpark implements ArmIO{

    private SparkMax motor;
    private SparkMaxConfig config;

    private AbsoluteEncoder encoder;

    private SparkClosedLoopController controller;

    public ArmIOSpark(){

        motor = new SparkMax(ArmConstants.canID, MotorType.kBrushless);

        encoder = motor.getAbsoluteEncoder();

        controller = motor.getClosedLoopController();


        config
            .inverted(ArmConstants.isInverted)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(ArmConstants.currentLimit)
            .voltageCompensation(ArmConstants.voltageCompensation);
        
        
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(
                ArmConstants.kP,
                ArmConstants.kI,
                ArmConstants.kD
            )
            .feedForward
                .kS(ArmConstants.kS);
        
        config.absoluteEncoder
            .inverted(ArmConstants.isAbsoluteInverted)
            .zeroOffset(ArmConstants.zeroOffset)
            .positionConversionFactor(ArmConstants.positionConversionFactor)
            .velocityConversionFactor(ArmConstants.absoluteEncoderVelocityConversionFactor);
        
        
        config.signals
                .appliedOutputPeriodMs(20) // See what voltage motor is using
                .outputCurrentPeriodMs(100) // Catch current spike
                .busVoltagePeriodMs(100) // Monitor battery voltage
                .faultsAlwaysOn(true)
                .faultsPeriodMs(1000) // Updates once a second, faults don't need hihg update rate
                .iAccumulationAlwaysOn(true) // tracks the integral accumulator for PID
                .motorTemperaturePeriodMs(1000)
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderPositionPeriodMs(20)
                .primaryEncoderVelocityAlwaysOn(true)
                .primaryEncoderVelocityPeriodMs(20) // Updates at robot loop rate
                .warningsAlwaysOn(true)
                .warningsPeriodMs(1000); // Updates once a second, warnings don't need high update rates
        
        motor.clearFaults();

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
    }

    public void updateInputs(ArmIOInputs inputs){

    }

    // @Override why does doing @Override give error? <- Question(I might've asked it before but I forgot)
    public void runPercentPower(double percent){
        motor.set(percent);
    }

    public void runVoltage(double voltage){
        motor.setVoltage(voltage);
    }

    public void runPosition(double position){
        controller.setSetpoint(position, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    public void stop(){
        motor.stopMotor();
    }

    public void setPID(double kP, double kI, double kD){
        config.closedLoop
            .pid(
                kP,
                kI,
                kD);
        
        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setkS(double kS){
        config.closedLoop.feedForward
            .kS(kS);

        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setBrakeMode(IdleMode mode){
        config.idleMode(mode);

        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void reset(){
        motor.clearFaults();
        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    }

}
