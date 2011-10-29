/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package org.first.team342;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.SimpleRobot;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the SimpleRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Team342Robot extends SimpleRobot {

    public static final int DEFAULT_MODULE_SLOT = 4;
    // Drive Motor Constants.
    public static final int PWM_CHANNEL_LEFT_FRONT = 1;
    public static final int PWM_CHANNEL_LEFT_REAR = 2;
    public static final int PWM_CHANNEL_RIGHT_FRONT = 3;
    public static final int PWM_CHANNEL_RIGHT_REAR = 4;
    // Arm Motor Constants.
    public static final int PWM_CHANNEL_GRIPPER_TOP = 6;
    public static final int PWM_CHANNEL_GRIPPER_BOTTOM = 7;
    public static final int PWM_CHANNEL_ARM_MOTOR = 10;
    //minibot
    public static final int PWM_CHANNEL_MINIBOT_ARM_RELEASE = 8;
    public static final int PWM_CHANNEL_MINIBOT_RELEASE = 9;
    public static final int DIO_CHANNEL_ARM_LIMIT_BOTTOM = 1;
    public static final int DIO_CHANNEL_ARM_LIMIT_TOP = 2;
    // Light Sensor Constants.
    public static final int DIO_CHANNEL_LIGHT_SENSOR_LEFT = 11;
    public static final int DIO_CHANNEL_LIGHT_SENSOR_CENTER = 10;
    public static final int DIO_CHANNEL_LIGHT_SENSOR_RIGHT = 9;
    //Arm limit switches
    public static final int DIO_CHANNEL_ARM_LIMIT = 1;
    // Joystick Constants.
    public static final int BUTTON_ROTATE_UP = 5;
    public static final int BUTTON_ROTATE_DOWN = 3;
    public static final int BUTTON_PULL_IN = 2;
    //public static final int BUTTON_SPIT = 8;
    public static final int BUTTON_DEPLOY_MINIBOT_ARM = 2;
    public static final int BUTTON_DEPLOY_MINIBOT = 3;
    //Joystick ports
    public static final int JOYSTICK_DRIVE_CONTROL = 1;
    public static final int JOYSTICK_ARM_CONTROL = 2;
    private RobotDrive drive;
    private Joystick driveController;
    private Joystick armController;
    private SpeedController leftFront;
    private SpeedController leftRear;
    private SpeedController rightFront;
    private SpeedController rightRear;
    private SpeedController armMotor;
    private SpeedController topGripper;
    private SpeedController bottomGripper;
    private Servo releaseArm;
    private Servo releaseBot;
    private DigitalInput leftSensor;
    private DigitalInput rightSensor;
    private DigitalInput centerSensor;
    private DigitalInput limitSwitch;
    private AutonomousGripperThread gripperRunnable;
    //driverstation refrence
    private DriverStation driversStation;

    public Team342Robot() {
        super();
        //init light sensors
        this.leftSensor = new DigitalInput(DEFAULT_MODULE_SLOT, DIO_CHANNEL_LIGHT_SENSOR_LEFT);
        this.rightSensor = new DigitalInput(DEFAULT_MODULE_SLOT, DIO_CHANNEL_LIGHT_SENSOR_RIGHT);
        this.centerSensor = new DigitalInput(DEFAULT_MODULE_SLOT, DIO_CHANNEL_LIGHT_SENSOR_CENTER);
        //init limit switch
        this.limitSwitch = new DigitalInput(DEFAULT_MODULE_SLOT, DIO_CHANNEL_ARM_LIMIT);
        //joysticks
        this.driveController = new Joystick(JOYSTICK_DRIVE_CONTROL);
        this.armController = new Joystick(JOYSTICK_ARM_CONTROL);

        this.releaseArm = new Servo(DEFAULT_MODULE_SLOT, PWM_CHANNEL_MINIBOT_ARM_RELEASE);
        this.releaseBot = new Servo(DEFAULT_MODULE_SLOT, PWM_CHANNEL_MINIBOT_RELEASE);

        this.leftFront = new Jaguar(DEFAULT_MODULE_SLOT, PWM_CHANNEL_LEFT_FRONT);
        this.leftRear = new Jaguar(DEFAULT_MODULE_SLOT, PWM_CHANNEL_LEFT_REAR);
        this.rightFront = new Jaguar(DEFAULT_MODULE_SLOT, PWM_CHANNEL_RIGHT_FRONT);
        this.rightRear = new Jaguar(DEFAULT_MODULE_SLOT, PWM_CHANNEL_RIGHT_REAR);

        this.armMotor = new Victor(DEFAULT_MODULE_SLOT, PWM_CHANNEL_ARM_MOTOR);
        this.topGripper = new Victor(DEFAULT_MODULE_SLOT, PWM_CHANNEL_GRIPPER_TOP);
        this.bottomGripper = new Victor(DEFAULT_MODULE_SLOT, PWM_CHANNEL_GRIPPER_BOTTOM);

        this.drive = new RobotDrive(this.leftFront, this.leftRear, this.rightFront, this.rightRear);
        this.drive.setInvertedMotor(RobotDrive.MotorType.kRearRight, true);
        this.drive.setInvertedMotor(RobotDrive.MotorType.kFrontRight, true);
    }

    /**
     * This function is called once each time the robot enters autonomous mode.
     */
    public void autonomous() {
        System.out.println("In Autonomous");

        this.gripperRunnable = new AutonomousGripperThread(topGripper, bottomGripper);
        Thread gripperThread = new Thread(gripperRunnable, "Gripper Thread");
        gripperThread.start();

        while (isAutonomous() && isEnabled()) {
            boolean sensors[] = {!rightSensor.get(), !centerSensor.get(), !leftSensor.get()};

            if (!limitSwitch.get()) {
                this.armMotor.set(0.7);
            } else {
                this.armMotor.set(0.2);
            }

            // drive control.
            switch (this.binaryMagic(sensors)) {
                case 0:
                    this.drive.tankDrive(0, 0);
                    System.out.println("Case 0");
                    break;
                case 1:
                    this.drive.tankDrive(0.6, -0.4);
                    System.out.println("Case 1");
                    break;
                case 2:
                    this.drive.tankDrive(.5, -.5);
                    System.out.println("Case 2");
                    break;
                case 3:
                    this.drive.tankDrive(0.6, -0.5);
                    System.out.println("Case 3");
                    break;
                case 4:
                    this.drive.tankDrive(0.4, -0.6);
                    System.out.println("Case 4");
                    break;
                case 5:
                    //cry ??????? 
                    System.out.println("Case 5");
                    break;
                case 6:
                    this.drive.tankDrive(0.5, -0.6);
                    System.out.println("Case 6");
                    break;
                case 7:
                    this.drive.tankDrive(0.0, 0.0);
                    System.out.println("Case 7");
                    this.topGripper.set(0.5);
                    this.bottomGripper.set(0.5);
                    break;
                default:
                    this.drive.tankDrive(0.0, 0.0);
                    break;
            }
        }

        Timer.delay(
                0.005);
    }

    /**
     * This function is called once each time the robot enters operator control.
     */
    public void operatorControl() {
        System.out.println("Is Operator Control: " + isOperatorControl());
        System.out.println("Is Enabled: " + isEnabled());
        while (isOperatorControl() && isEnabled()) {
            System.out.println("Limit switch: " + limitSwitch.get());
            double x = this.driveController.getX();
            double y = this.driveController.getY();
            double armValue = this.armController.getY() * -1;
            double rotation = 0.0;
            if (this.driveController.getRawButton(5) && this.driveController.getRawButton(6)) {
                rotation = 0.0;
            } else if (this.driveController.getRawButton(5)) {
                rotation = -0.75;
            } else if (this.driveController.getRawButton(6)) {
                rotation = 0.75;
            }
            this.drive.mecanumDrive_Cartesian(x, y, rotation, 0);
            this.armMotor.set(armValue);

            if (this.armController.getRawButton(BUTTON_PULL_IN)) {
                this.topGripper.set(-0.5);
                this.bottomGripper.set(-0.5);
            } else if (this.armController.getRawButton(BUTTON_ROTATE_UP)) {
                this.topGripper.set(-0.5);
                this.bottomGripper.set(0.5);
            } else if (this.armController.getRawButton(BUTTON_ROTATE_DOWN)) {
                this.topGripper.set(0.5);
                this.bottomGripper.set(-0.5);
            } else if (this.armController.getTrigger()) {
                this.topGripper.set(0.5);
                this.bottomGripper.set(0.5);
            } else {
                this.topGripper.set(0.0);
                this.bottomGripper.set(0.0);
            }

            if (this.driveController.getRawButton(BUTTON_DEPLOY_MINIBOT_ARM) && this.driveController.getRawButton(BUTTON_DEPLOY_MINIBOT)) {
                this.releaseBot.setAngle(0.0);
            } else if (this.driveController.getRawButton(BUTTON_DEPLOY_MINIBOT_ARM)) {
                this.releaseArm.setAngle(170.0);
            } else {
                if (this.releaseArm.get() > 0.0) {
                    this.releaseArm.set(0.0);
                }
                if (this.releaseBot.get() < 170.0) {
                    this.releaseBot.set(170.0);
                }
            }

            Timer.delay(0.005);
        }
    }

    public void disabled() {
        while (isDisabled()) {
            this.armMotor.set(0.0);
            this.drive.stopMotor();
            this.topGripper.set(0.0);
            this.bottomGripper.set(0.0);
            
            if (this.gripperRunnable != null) {
                this.gripperRunnable.terminate();
            }
            
            Timer.delay(0.005);

        }
    }

    public int binaryMagic(boolean[] input) {
        int output = 0;
        int placeValue = 1;
        for (int i = 0; i < input.length; i++) {
            if (input[i]) {
                output += placeValue;
            }

            placeValue *= 2;
        }
        return output;
    }
}
