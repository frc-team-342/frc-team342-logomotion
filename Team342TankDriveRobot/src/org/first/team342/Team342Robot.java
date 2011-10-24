
/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package org.first.team342;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SimpleRobot;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the SimpleRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Team342Robot extends SimpleRobot {

    // Robot Constants.
    public static final int DIGITAL_MODULE_SLOT = 4;
    // Speed Controller Constants.
    public static final int PWM_CHANNEL_LEFT_FRONT = 1;
    public static final int PWM_CHANNEL_RIGHT_FRONT = 2;
    public static final int PWM_CHANNEL_LEFT_REAR = 3;
    public static final int PWM_CHANNEL_RIGHT_REAR = 4;
    // Light Sensor Constants.
    public static final int DIO_CHANNEL_LIGHT_SENSOR_LEFT = 3;
    public static final int DIO_CHANNEL_LIGHT_SENSOR_CENTER = 4;
    public static final int DIO_CHANNEL_LIGHT_SENSOR_RIGHT = 5;
    // Joystick constants.
    public static final int JOYSTICK_PORT_LEFT = 0;
    public static final int JOYSTICK_PORT_RIGHT = 1;
    public static final int JOYSTICK_PORT_ARM = 2;
    // Instance variables.
    private Joystick leftJoystick;
    private Joystick rightJoystick;
    private Joystick armJoystick;
    private RobotDrive drive;
    //light sensors
    private DigitalInput leftSensor;
    private DigitalInput rightSensor;
    private DigitalInput centerSensor;

    public Team342Robot() {
        super();

        // Initialize Joysticks.
        // TODO: Create constants for the Joystick ports.
        this.leftJoystick = new Joystick(JOYSTICK_PORT_LEFT);
        this.rightJoystick = new Joystick(JOYSTICK_PORT_RIGHT);
        this.armJoystick = new Joystick(JOYSTICK_PORT_ARM);

        // Initialize Speedcontrollers.
        SpeedController leftFront = new Jaguar(DIGITAL_MODULE_SLOT, PWM_CHANNEL_LEFT_FRONT);
        SpeedController rightFront = new Jaguar(DIGITAL_MODULE_SLOT, PWM_CHANNEL_RIGHT_FRONT);
        SpeedController leftRear = new Jaguar(DIGITAL_MODULE_SLOT, PWM_CHANNEL_LEFT_REAR);
        SpeedController rightRear = new Jaguar(DIGITAL_MODULE_SLOT, PWM_CHANNEL_RIGHT_REAR);

        //Initialize Light Sensors As DIO
        this.leftSensor = new DigitalInput(DIGITAL_MODULE_SLOT, DIO_CHANNEL_LIGHT_SENSOR_LEFT);
        this.rightSensor = new DigitalInput(DIGITAL_MODULE_SLOT, DIO_CHANNEL_LIGHT_SENSOR_RIGHT);
        this.centerSensor = new DigitalInput(DIGITAL_MODULE_SLOT, DIO_CHANNEL_LIGHT_SENSOR_CENTER);
        // Initialize Robot Drive.
        this.drive = new RobotDrive(leftFront, leftRear, rightFront, rightRear);
    }

    /**
     * This function is called once each time the robot enters autonomous mode.
     */
    public void autonomous() {
        System.out.println("In Autonomous Mode");

        while (isAutonomous() && isEnabled()) {
            boolean sensors[] = {rightSensor.get(), centerSensor.get(), leftSensor.get()};

            int value = binaryMagic(sensors);

            switch (value) {
                case 0:
                    break;
                case 1:
                    break;
                case 2:
                    break;
                case 3:
                    break;
                case 4:
                    break;
                case 5:
                    break;
                case 6:
                    break;
                case 7:
                    break;
                default:
                    break;
            }
        }

        Timer.delay(0.005);
    }

    /**
     * This function is called once each time the robot enters operator control.
     */
    public void operatorControl() {
        while (isOperatorControl() && isEnabled()) {
            this.drive.tankDrive(this.leftJoystick, this.rightJoystick);
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
