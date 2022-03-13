package com.swervedrivespecialties.swervelib.ctre;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.swervedrivespecialties.swervelib.AbsoluteEncoder;
import com.swervedrivespecialties.swervelib.AbsoluteEncoderFactory;

import edu.wpi.first.wpilibj.DriverStation;

public class CanCoderFactoryBuilder {
    private Direction direction = Direction.COUNTER_CLOCKWISE;
    private int periodMilliseconds = 10;

    public CanCoderFactoryBuilder withReadingUpdatePeriod(int periodMilliseconds) {
        this.periodMilliseconds = periodMilliseconds;
        return this;
    }

    public CanCoderFactoryBuilder withDirection(Direction direction) {
        this.direction = direction;
        return this;
    }

    public AbsoluteEncoderFactory<CanCoderAbsoluteConfiguration> build() {
        return configuration -> {
            CANCoderConfiguration config = new CANCoderConfiguration();
            config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
            config.magnetOffsetDegrees = Math.toDegrees(configuration.getOffset());
            config.sensorDirection = direction == Direction.CLOCKWISE;
            config.initializationStrategy = configuration.getInitStrategy();

            CANCoder encoder = new CANCoder(configuration.getId());
            CtreUtils.checkCtreError(encoder.configAllSettings(config, 250), "Failed to configure CANCoder");

            CtreUtils.checkCtreError(encoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, periodMilliseconds, 250), "Failed to configure CANCoder update rate");

            return new EncoderImplementation(encoder);
        };
    }

    private static class EncoderImplementation implements AbsoluteEncoder {
        private final int ATTEMPTS = 10;

        private final CANCoder encoder;

        private EncoderImplementation(CANCoder encoder) {
            this.encoder = encoder;
        }

        @Override
        public double getAbsoluteAngle() {
            double angle = Math.toRadians(encoder.getPosition());

            ErrorCode code = encoder.getLastError();

            for (int i = 0; i < ATTEMPTS; i++) {
                if (code == ErrorCode.OK) break;            
                try {
                    Thread.sleep(10);
                    DriverStation.reportError(String.format("%s: %s", "Message from Eric", "attempts: " + ATTEMPTS), false);
                } catch (InterruptedException e) { }
                angle = Math.toRadians(encoder.getPosition());
                code = encoder.getLastError();
            }

            CtreUtils.checkCtreError(code, "Failed to retrieve CANcoder "+encoder.getDeviceID()+" absolute position after "+ATTEMPTS+" tries");

            angle %= 2.0 * Math.PI;
            if (angle < 0.0) {
                angle += 2.0 * Math.PI;
            }

            return angle;
        }
    }

    public enum Direction {
        CLOCKWISE,
        COUNTER_CLOCKWISE
    }
}