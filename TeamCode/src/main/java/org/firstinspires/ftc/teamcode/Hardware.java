package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.util.ArrayList;
import java.util.Arrays;

/**
 * Created by okosa on 9/9/2017.
 */

public class Hardware {
    //Configuration Constants
    private static final String LEFT_FRONT_MOTOR = "lf";
    private static final String RIGHT_FRONT_MOTOR = "rf";
    private static final String LEFT_REAR_MOTOR = "lr";
    private static final String RIGHT_REAR_MOTOR = "rr";
    private static final String LEFT_COLLECTOR_MOTOR = "lc";
    private static final String RIGHT_COLLECTOR_MOTOR = "rc";
    private static final String LIFT_MOTOR = "lm";
    private static final String JEWEL_SERVO = "js";
    private static final String GRAB_BOTTOM_SERVO = "gbs";
    private static final String GRAB_TOP_SERVO = "gts";
    private static final String TILT_SERVO = "ts";
    private static final String FLIP_SERVO = "fs";
    private static final String RELIC_GRAB_SERVO = "rgs";
    private static final String RELIC_TILT_SERVO = "rts";
    private static final String RELIC_ARM_TILT_SERVO = "rats";
    private static final String RELIC_ARM_EXTEND_SERVO = "raes";


    private static final String COLOR_SENSOR = "cs";
    private static final String LIGHT_SENSOR = "ls";
    final static String IMU = "imu";
    private static final String TOUCH_SENSOR_TOP = "tst";
    private static final String TOUCH_SENSOR_BOTTOM = "tsb";

    private static final DcMotor.Direction LEFT_FRONT_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;
    private static final DcMotor.Direction RIGHT_FRONT_MOTOR_DIRECTION = DcMotorSimple.Direction.REVERSE;
    private static final DcMotor.Direction LEFT_REAR_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;
    private static final DcMotor.Direction RIGHT_REAR_MOTOR_DIRECTION = DcMotorSimple.Direction.REVERSE;
    private static final DcMotor.Direction LEFT_COLLECTOR_MOTOR_DIRECTION = DcMotorSimple.Direction.REVERSE;
    private static final DcMotor.Direction RIGHT_COLLECTOR_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;
    private static final DcMotor.Direction LIFT_MOTOR_DIRECTION = DcMotorSimple.Direction.REVERSE;

    private static final String ACTIVE_CIPHER_FILE_NAME = "ActiveCipher";


    public static final double GRAB_BOTTOM_SERVO_GRAB = 0.645; //new
    public static final double GRAB_BOTTOM_SERVO_RELEASE = 0.454444444;
    public static final double GRAB_TOP_SERVO_GRAB = 0.583333333;
    public static final double GRAB_TOP_SERVO_RELEASE = 0.748333333;
    public static final double FLIP_SERVO_UP = .07;
    public static final double FLIP_SERVO_DOWN = .98;
    public static final double RELIC_GRAB_SERVO_GRAB = .52;
    public static final double RELIC_GRAB_SERVO_RELEASE = .35;
    public static final double RELIC_TILT_SERVO_UPPER_LIMIT = .97;
    public static final double RELIC_TILT_SERVO_LOWER_LIMIT = .1;
    public double relicTiltServoValue = .5;
    ;
    public static final double RELIC_TILT_SERVO_90_DEGREE_VALUE = 0.491111111;
    public static final double RELIC_TILT_SERVO_0_DEGREE_VALUE = 0.968888889;
    public static final double RELIC_ARM_TILT_SERVO_UPPER_LIMIT = .78;
    public static final double RELIC_ARM_TILT_SERVO_LOWER_LIMIT = .16;
    public double relicArmTiltServoValue = .5;
    double relicArmTiltSpeed = .07602; //change in position per second
    public static final double RELIC_ARM_TILT_SERVO_0_DEGREE_VALUE = 0.480555556;
    public static final double RELIC_ARM_TILT_SERVO_90_DEGREE_VALUE = 0.698888889;
    public static final double RELIC_ARM_EXTEND_SERVO_LOWER_LIMIT = .03;
    public static final double RELIC_ARM_EXTEND_SERVO_UPPER_LIMIT = .48;
    double relicArmExtendSpeed = .0625; //change in position per second
    public double relicArmExtendServoValue = .5;
    public static final double TILT_SERVO_UP = 0.724444444 ;
    public static final double TILT_SERVO_DOWN = 0.956;
    public static final double JEWEL_SERVO_DOWN = .7;
    public static final double JEWEL_SERVO_UP = .204;
    private static final double GREY_VALUE = 4;
    private static final double BROWN_VALUE = 2;
    private static final double RED_THRESHOLD = 10;
    private static final double BLUE_THRESHOLD = 10;


    //Hardware Devices
    DcMotor leftFrontMotor;
    DcMotor rightFrontMotor;
    DcMotor leftRearMotor;
    DcMotor rightRearMotor;
    DcMotor leftCollectorMotor;
    DcMotor rightCollectorMotor;
    DcMotor liftMotor;
    Servo jewelServo;
    Servo tiltServo;
    Servo grabBottomServo;
    Servo grabTopServo;
    Servo flipServo;
    Servo relicGrabServo;
    Servo relicTiltServo;
    Servo relicArmTiltServo;
    Servo relicArmExtendServo;
    BNO055IMU imu;
    OpticalDistanceSensor lightSensor;
    DigitalChannel touchSensorTop;
    DigitalChannel touchSensorBottom;
    LynxI2cColorRangeSensor colorSensor;

    //Local Variables
    private HardwareMap hardwareMap;
    private VuforiaLocalizer vuforia;
    private VuforiaTrackable relicTemplate;
    ArrayList<Integer> possibleCiphers;

    public boolean robotOrientationForward;
    public boolean glyphsFlippedUp;
    public double speedMultiplier = 1;
    boolean isRelicTiltParallelToGround = false;

    enum ColorDetected {
        BLUE("Blue"),
        RED("Red"),
        NONE("None");

        String description;

        public String toString() {
            return description;
        }

        ColorDetected(String description) {
            this.description = description;
        }
    }

    public enum Glyph {
        GREY("Grey"),
        BROWN("Brown"),
        NONE("None");

        String description;

        public String toString() {
            return description;
        }

        Glyph(String description) {
            this.description = description;
        }
    }


    Thread slowFlip = null;
    boolean flipServoIsUp = true;

    public void setFlipServoUp(boolean flipUp) {
        final double target;
        if (isApproximatelyEqual(tiltServo.getPosition(), TILT_SERVO_UP) && touchSensorBottom.getState() == true) {
            if (flipUp) {
                target = FLIP_SERVO_UP;
                flipServoIsUp = true;
            } else {
                target = FLIP_SERVO_DOWN;
                flipServoIsUp = false;
            }/**
            if (slowFlip != null && slowFlip.isAlive()) {
                slowFlip.interrupt();
            }
            slowFlip = new Thread() {
                @Override
                public void run() {
                    super.run();
                    long lastTime = System.currentTimeMillis();
                    double duration = Math.abs(target - flipServo.getPosition()) * 1500;
                    int numOfIncrements = (int) (duration / 100);
                    double incrementDistance = (target - flipServo.getPosition()) / numOfIncrements;
                    for (int i = 0; i < numOfIncrements; i++) {
                        while (lastTime + 100 > System.currentTimeMillis()) {
                            try {
                                Thread.sleep(10);
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                            }
                        }
                        lastTime += 100;
                        flipServo.setPosition(flipServo.getPosition() + incrementDistance);
                        if (flipServo.getPosition() > FLIP_SERVO_DOWN)
                            flipServo.setPosition(FLIP_SERVO_DOWN);
                        if (flipServo.getPosition() < FLIP_SERVO_UP)
                            flipServo.setPosition(FLIP_SERVO_UP);
                    }
                    flipServo.setPosition(target);
                }
            };
            slowFlip.start();
             */
            flipServo.setPosition(target);
        }
    }

    private void initialize() {
        leftFrontMotor = getHardwareDevice(DcMotor.class, LEFT_FRONT_MOTOR);
        rightRearMotor = getHardwareDevice(DcMotor.class, RIGHT_REAR_MOTOR);
        leftRearMotor = getHardwareDevice(DcMotor.class, LEFT_REAR_MOTOR);
        rightFrontMotor = getHardwareDevice(DcMotor.class, RIGHT_FRONT_MOTOR);
        rightCollectorMotor = getHardwareDevice(DcMotor.class, RIGHT_COLLECTOR_MOTOR);
        leftCollectorMotor = getHardwareDevice(DcMotor.class, LEFT_COLLECTOR_MOTOR);
        liftMotor = getHardwareDevice(DcMotor.class, LIFT_MOTOR);
        jewelServo = getHardwareDevice(Servo.class, JEWEL_SERVO);
        tiltServo = getHardwareDevice(Servo.class, TILT_SERVO);
        flipServo = getHardwareDevice(Servo.class, FLIP_SERVO);
        grabBottomServo = getHardwareDevice(Servo.class, GRAB_BOTTOM_SERVO);
        grabTopServo = getHardwareDevice(Servo.class, GRAB_TOP_SERVO);
        relicGrabServo = getHardwareDevice(Servo.class, RELIC_GRAB_SERVO);
        relicTiltServo = getHardwareDevice(Servo.class, RELIC_TILT_SERVO);
        relicArmTiltServo = getHardwareDevice(Servo.class, RELIC_ARM_TILT_SERVO);
        relicArmExtendServo = getHardwareDevice(Servo.class, RELIC_ARM_EXTEND_SERVO);
        touchSensorBottom = getHardwareDevice(DigitalChannel.class, TOUCH_SENSOR_BOTTOM);
        touchSensorTop = getHardwareDevice(DigitalChannel.class, TOUCH_SENSOR_TOP);
        colorSensor = getHardwareDevice(LynxI2cColorRangeSensor.class, COLOR_SENSOR);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = getHardwareDevice(BNO055IMU.class, IMU);
        imu.initialize(parameters);
        //imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        leftFrontMotor.setDirection(LEFT_FRONT_MOTOR_DIRECTION);
        rightFrontMotor.setDirection(RIGHT_FRONT_MOTOR_DIRECTION);
        leftRearMotor.setDirection(LEFT_REAR_MOTOR_DIRECTION);
        rightRearMotor.setDirection(RIGHT_REAR_MOTOR_DIRECTION);
        leftCollectorMotor.setDirection(LEFT_COLLECTOR_MOTOR_DIRECTION);
        rightCollectorMotor.setDirection(RIGHT_COLLECTOR_MOTOR_DIRECTION);
        liftMotor.setDirection(LIFT_MOTOR_DIRECTION);

        tiltServo.setPosition(TILT_SERVO_DOWN);
        flipServo.setPosition(FLIP_SERVO_UP);
        grabBottomServo.setPosition(GRAB_BOTTOM_SERVO_RELEASE);
        grabTopServo.setPosition(GRAB_TOP_SERVO_RELEASE);
        relicArmExtendServo.setPosition(RELIC_ARM_EXTEND_SERVO_UPPER_LIMIT);

    }

    //Initialize IMU
    private void initializeImuParameters() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        imu.initialize(parameters);
    }

    public void setDriveTrainRunMode(DcMotor.RunMode runMode) {
        leftFrontMotor.setMode(runMode);
        rightFrontMotor.setMode(runMode);
        leftRearMotor.setMode(runMode);
        rightRearMotor.setMode(runMode);
    }


    public void drive(double forwardValue, double sideValue, double rotationValue) {
        double leftFrontPower;
        double leftRearPower;
        double rightFrontPower;
        double rightRearPower;

        if (robotOrientationForward == false) {
            forwardValue = -forwardValue;
            sideValue = -sideValue;
        }

        //forwardValue = powWithoutLosingNegative(forwardValue, 1.65);
        //sideValue = powWithoutLosingNegative(sideValue, 1.65);
        //rotationValue = powWithoutLosingNegative(rotationValue, 1.65);

        forwardValue *= speedMultiplier;
        sideValue *= speedMultiplier;
        rotationValue *= speedMultiplier;

        leftFrontPower = forwardValue + sideValue + rotationValue;
        leftRearPower = forwardValue - sideValue + rotationValue;
        rightFrontPower = forwardValue - sideValue - rotationValue;
        rightRearPower = forwardValue + sideValue - rotationValue;


        double max = Double.MIN_VALUE;
        if (Math.abs(leftFrontPower) > max)
            max = Math.abs(leftFrontPower);
        if (Math.abs(leftRearPower) > max)
            max = Math.abs(leftRearPower);
        if (Math.abs(rightFrontPower) > max)
            max = Math.abs(rightFrontPower);
        if (Math.abs(rightRearPower) > max)
            max = Math.abs(rightRearPower);
        if (max > 1) {
            leftFrontPower /= max;
            leftRearPower /= max;
            rightFrontPower /= max;
            rightRearPower /= max;
        }

        leftFrontMotor.setPower(leftFrontPower);
        leftRearMotor.setPower(leftRearPower);
        rightFrontMotor.setPower(rightFrontPower);
        rightRearMotor.setPower(rightRearPower);
    }

    public void setCollectorPower(double power) {
        leftCollectorMotor.setPower(power);
        rightCollectorMotor.setPower(power);
    }

    public double turn(double finalHeading) {
        while (finalHeading < -180) {
            finalHeading += 360;
        }
        while (finalHeading > 180) {
            finalHeading -= 360;
        }

        double heading = getAngle();


        if (heading > 180) { // convert 0 - 360 range of heading to -180 - 180
            heading += 180;
            heading %= 360;
            heading -= 180;
        }

        double turnPower;
        if (Math.abs(heading - finalHeading) > 120) {
            turnPower = 1;
        } else if (Math.abs(heading - finalHeading) > 90) {
            turnPower = .7;
        } else if (Math.abs(heading - finalHeading) > 40) {
            turnPower = .5;
        } else if (Math.abs(heading - finalHeading) > 30) {
            turnPower = .4;
        } else if (Math.abs(heading - finalHeading) > 25) {
            turnPower = .4;
        } else if (Math.abs(heading - finalHeading) > 20) {
            turnPower = .3;

        } else if (Math.abs(heading - finalHeading) > 15) {
            turnPower = .2;

        } else if (Math.abs(heading - finalHeading) > 10) {
            turnPower = .1;
        } else if (Math.abs(heading - finalHeading) > 5) {
            turnPower = .05;
        } else {
            turnPower = .02;
        }

        if (Math.abs(heading - finalHeading) < 1
                ) {
            return 0;
        }
        if (heading <= 180 && heading >= 0 && finalHeading <= 180 && finalHeading >= 0) {
            if (finalHeading > heading) {
                return turnPower;
            } else {
                return -turnPower;
            }
        } else if (heading >= -180 && heading <= 0 && finalHeading >= -180 && finalHeading <= 0) {
            if (finalHeading > heading) {
                return turnPower;
            } else {
                return -turnPower;
            }
        } else if (heading >= -180 && heading <= 0 && finalHeading <= 180 && finalHeading >= 0) {
            if (finalHeading - heading < (heading + 360) - finalHeading) {
                return turnPower;
            } else {
                return -turnPower;
            }
        } else if (heading <= 180 && heading >= 0 && finalHeading >= -180 && finalHeading <= 0) {
            if (heading - finalHeading > (finalHeading + 360) - heading) {
                return turnPower;
            } else {
                return -turnPower;
            }
        }
        return .00005;
    }

    public void stop() {
        drive(0, 0, 0);
    }

    //Find angle from gyro
    double getAngle() {
        return -imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES)
                .toAngleUnit(AngleUnit.DEGREES).firstAngle;
    }

    Hardware(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        initialize();
        //initializeImuParameters();
        possibleCiphers = new ArrayList<Integer>(Arrays.asList(0, 1, 2, 3, 4, 5));
        robotOrientationForward = true;
        glyphsFlippedUp = true;
    }

    private <T> T getHardwareDevice(Class<? extends T> classOrInterface, String deviceName) {
        try {
            return hardwareMap.get(classOrInterface, deviceName);
        } catch (Exception e) {
            RobotLog.e("Missing " + classOrInterface.toString() + " " + deviceName);
            return null;
        }
    }

    public void initializeVuforia() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AdYQ1UT/////AAAAGduICslDnEnttzNkRGI2spxKjPBtdo/7cWrldv0MqHAmbK0Fyjw65zsW4JCkN6GRGjkwkLLX4kMkfjY2j/7K9K74AA1RRn1QaxpqfHqWfPXudWPzt4Y3PaLHK5c6ge6m6PyDYTZMxZmgb2jS5JaR0KPUf8Vmu1ysEOZfSNcSC20G56QRO/9VpJRrfetFMlsDiAwmsj+muYdKN5fwRCW3N8KK7CVD2ad9mXKvv45082O9PL0zXxq1vYPeKmn/27V1ihKOI0JHL5vEIeN3XeA56SM1f7yiLk2LFmkY+sM6K+HaDL+wLIulcuUIidqZ0xwKFFHCPjssVaZ25RtHYUY4nIvS+LJdzO+FdTYNDqtOn95Q";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");
        relicTrackables.activate();
    }

    public RelicRecoveryVuMark getPictograph() {
        return RelicRecoveryVuMark.from(relicTemplate);
    }

    public ColorDetected getColorDetected() {
        if (colorSensor.blue() > BLUE_THRESHOLD && colorSensor.blue() > colorSensor.red()) {
            return ColorDetected.BLUE;
        }
        if (colorSensor.red() > RED_THRESHOLD && colorSensor.blue() < colorSensor.red()) {
            return ColorDetected.RED;
        }
        return ColorDetected.NONE;
    }

    public void setTiltServoPositionUp(boolean tiltServoPositionUp) {
        if (touchSensorBottom.getState() == false) {
            if (tiltServoPositionUp) {
                tiltServo.setPosition(TILT_SERVO_UP);
                grabBottomServo.setPosition(GRAB_BOTTOM_SERVO_GRAB);
                grabTopServo.setPosition(GRAB_TOP_SERVO_GRAB);
            } else if (flipServoIsUp) {
                tiltServo.setPosition(TILT_SERVO_DOWN);
            }
        }
    }

    Double previousRelicArmTiltTime = null;
    Double relicArmServoTiltPosition = null;

    void incrementRelicArmTiltPosition(double x, double gameTime) {
        if (previousRelicArmTiltTime == null || relicArmServoTiltPosition == null) {
            previousRelicArmTiltTime = gameTime;
            relicArmServoTiltPosition = relicArmTiltServo.getPosition();
        } else {
            relicArmServoTiltPosition += x * relicArmTiltSpeed * (gameTime - previousRelicArmTiltTime);
            if (relicArmServoTiltPosition < RELIC_ARM_TILT_SERVO_LOWER_LIMIT) {
                relicArmServoTiltPosition = RELIC_ARM_TILT_SERVO_LOWER_LIMIT;
            } else if (relicArmServoTiltPosition > RELIC_ARM_TILT_SERVO_UPPER_LIMIT)
                relicArmServoTiltPosition = RELIC_ARM_TILT_SERVO_UPPER_LIMIT;
            previousRelicArmTiltTime = gameTime;
            relicArmTiltServo.setPosition(relicArmServoTiltPosition);
        }
    }

    Double previousRelicArmExtendTime = null;
    Double relicArmServoExtendPosition = null;

    void incrementRelicArmExtendPosition(double x, double gameTime) {
        if (previousRelicArmExtendTime == null || relicArmServoExtendPosition == null) {
            previousRelicArmExtendTime = gameTime;
            relicArmServoExtendPosition = relicArmExtendServo.getPosition();
        } else {
            relicArmServoExtendPosition += x * relicArmExtendSpeed * (gameTime - previousRelicArmExtendTime);
            if (relicArmServoExtendPosition < RELIC_ARM_EXTEND_SERVO_LOWER_LIMIT) {
                relicArmServoExtendPosition = RELIC_ARM_EXTEND_SERVO_LOWER_LIMIT;
            } else if (relicArmServoExtendPosition > RELIC_ARM_EXTEND_SERVO_UPPER_LIMIT) {
                relicArmServoExtendPosition = RELIC_ARM_EXTEND_SERVO_UPPER_LIMIT;
            }
            previousRelicArmExtendTime = gameTime;
            relicArmExtendServo.setPosition(relicArmServoExtendPosition);
        }
    }

    void setRelicTiltServoPosition() {
        double position;
        if (isRelicTiltParallelToGround) {
            position = (Range.scale(Range.scale(relicArmTiltServo.getPosition(),
                    RELIC_ARM_TILT_SERVO_0_DEGREE_VALUE, RELIC_ARM_TILT_SERVO_90_DEGREE_VALUE,
                    0, Math.PI / 4), 0, Math.PI / 4, RELIC_TILT_SERVO_0_DEGREE_VALUE,
                    RELIC_TILT_SERVO_90_DEGREE_VALUE));
        } else {
            position = (Range.scale(-Math.PI / 4 + Range.scale(relicArmTiltServo.getPosition(),
                    RELIC_ARM_TILT_SERVO_0_DEGREE_VALUE, RELIC_ARM_TILT_SERVO_90_DEGREE_VALUE,
                    0, Math.PI / 4), 0, Math.PI / 4, RELIC_TILT_SERVO_0_DEGREE_VALUE,
                    RELIC_TILT_SERVO_90_DEGREE_VALUE));

        }
        if (position > RELIC_TILT_SERVO_UPPER_LIMIT)
            position = (RELIC_TILT_SERVO_UPPER_LIMIT);
        else if (position < RELIC_TILT_SERVO_LOWER_LIMIT)
            position = (RELIC_TILT_SERVO_LOWER_LIMIT);
        relicTiltServo.setPosition(position);
    }

    public Glyph getGlyphDetected() {
        double lightDetected = lightSensor.getLightDetected();
        if (lightDetected < GREY_VALUE && lightDetected > GREY_VALUE - 0.2) {
            return Glyph.GREY;
        }
        if (lightDetected < BROWN_VALUE && lightDetected > BROWN_VALUE - 0.2) {
            return Glyph.BROWN;
        }
        return Glyph.NONE;
    }

    final public Glyph cipherPattern[][][] =
            {
                    { //Frog
                            {Glyph.GREY, Glyph.BROWN, Glyph.GREY},
                            {Glyph.BROWN, Glyph.GREY, Glyph.BROWN},
                            {Glyph.GREY, Glyph.BROWN, Glyph.GREY},
                            {Glyph.BROWN, Glyph.GREY, Glyph.BROWN}
                    },
                    { //Frog Inverse
                            {Glyph.BROWN, Glyph.GREY, Glyph.BROWN},
                            {Glyph.GREY, Glyph.BROWN, Glyph.GREY},
                            {Glyph.BROWN, Glyph.GREY, Glyph.BROWN},
                            {Glyph.GREY, Glyph.BROWN, Glyph.GREY}
                    },
                    { //Bird
                            {Glyph.GREY, Glyph.BROWN, Glyph.GREY},
                            {Glyph.BROWN, Glyph.GREY, Glyph.BROWN},
                            {Glyph.BROWN, Glyph.GREY, Glyph.BROWN},
                            {Glyph.GREY, Glyph.BROWN, Glyph.GREY}
                    },
                    { //Bird Inverse
                            {Glyph.BROWN, Glyph.GREY, Glyph.BROWN},
                            {Glyph.GREY, Glyph.BROWN, Glyph.GREY},
                            {Glyph.GREY, Glyph.BROWN, Glyph.GREY},
                            {Glyph.BROWN, Glyph.GREY, Glyph.BROWN},
                    },
                    { //Snake
                            {Glyph.BROWN, Glyph.GREY, Glyph.GREY},
                            {Glyph.BROWN, Glyph.BROWN, Glyph.GREY},
                            {Glyph.GREY, Glyph.BROWN, Glyph.BROWN},
                            {Glyph.GREY, Glyph.GREY, Glyph.BROWN},
                    },
                    { //Snake Inverse
                            {Glyph.GREY, Glyph.BROWN, Glyph.BROWN},
                            {Glyph.GREY, Glyph.GREY, Glyph.BROWN},
                            {Glyph.BROWN, Glyph.GREY, Glyph.GREY},
                            {Glyph.BROWN, Glyph.BROWN, Glyph.GREY},
                    }
            };
    Glyph activeCipher[][] =
            {
                    {Glyph.NONE, Glyph.NONE, Glyph.NONE},
                    {Glyph.NONE, Glyph.NONE, Glyph.NONE},
                    {Glyph.NONE, Glyph.NONE, Glyph.NONE},
                    {Glyph.NONE, Glyph.NONE, Glyph.NONE}
            };

    public void saveActiveCipherToFile() throws IOException {
        FileOutputStream fos = hardwareMap.appContext.openFileOutput(ACTIVE_CIPHER_FILE_NAME, hardwareMap.appContext.MODE_PRIVATE);
        ObjectOutputStream os = new ObjectOutputStream(fos);
        os.writeObject(activeCipher);
        os.close();
        fos.close();
    }

    public void loadActiveCipherFromFile() throws IOException, ClassNotFoundException {
        FileInputStream fis = hardwareMap.appContext.openFileInput(ACTIVE_CIPHER_FILE_NAME);
        ObjectInputStream is = new ObjectInputStream(fis);
        activeCipher = (Glyph[][]) is.readObject();
        is.close();
        fis.close();
    }

    void setLiftPower(double power) {
        if (touchSensorBottom.getState() == false && power < 0)
            liftMotor.setPower(0);
        else if ((isApproximatelyEqual(tiltServo.getPosition(), TILT_SERVO_DOWN) || touchSensorTop.getState() == false) && power > 0)
            liftMotor.setPower(0);
        else
            liftMotor.setPower(power);
    }

    void updatePossibleCiphers() {
        for (int i = possibleCiphers.size() - 1; i <= 0; i--) {
            if (!isCipherPossible(possibleCiphers.get(i))) {
                possibleCiphers.remove(i);
            }
        }
    }

    /**
     * int getNextColumn(Glyph glyph1, Glyph glyph2) {
     * for (int i = 0; i < possibleCiphers.size(); i++) {
     * for (int y = 0; y < cipherPattern[i][0].length; i++) {
     * for (int x = cipherPattern[i].length - 1; x >= 0; x--) {
     * if (activeCipher[y][x] == Glyph.NONE){
     * if (glyph1 == cipherPattern[i][y][x]){
     * if (glyph2 == Glyph.NONE){
     * <p>
     * }else if (x - 1 >= 0 && glyph2 == cipherPattern[i][y][x - 1])
     * }
     * }
     * }
     * }
     * }
     * }
     **/

    boolean isCipherPossible(int cipherNumber) {
        for (int j = cipherPattern[cipherNumber].length - 1; j >= 0; j--) {
            for (int k = 0; k < cipherPattern[cipherNumber][j].length; k++) {
                if (activeCipher[j][k] != Hardware.Glyph.NONE && activeCipher[j][k] != cipherPattern[cipherNumber][j][k]) {
                    return false;
                }
            }
        }
        return true;
    }

    static double powWithoutLosingNegative(double n, double power) {
        double result = Math.pow(n, power);
        if (result > 0 && n < 0)
            result = -result;
        return result;
    }

    static boolean isApproximatelyEqual(double x, double y){
        return Math.abs(x - y) < .02;
    }
}
