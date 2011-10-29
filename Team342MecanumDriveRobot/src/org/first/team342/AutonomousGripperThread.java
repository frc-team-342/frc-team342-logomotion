/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package org.first.team342;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;

/**
 *
 * @author abrightwell
 */
public class AutonomousGripperThread implements Runnable {

    private SpeedController topGripper;
    private SpeedController bottomGripper;
    private boolean terminate;

    public AutonomousGripperThread(SpeedController topGripper, SpeedController bottomGripper) {
        super();
        this.topGripper = topGripper;
        this.bottomGripper = bottomGripper;
        this.terminate = false;
    }

    public void run() {
        double initialTime = Timer.getFPGATimestamp();
        while (!terminate) {
            double currentTime = Timer.getFPGATimestamp();
            double elapsedTime = (currentTime - initialTime);
            System.out.println("Hi I am running! :)");
            if (elapsedTime < 4.25) {
                this.topGripper.set(0.0);
                this.bottomGripper.set(0.0);
            } else if (elapsedTime >= 4.25 && elapsedTime < 5.0) {
                this.topGripper.set(-0.5);
                this.bottomGripper.set(0.5);
            } else {
                this.topGripper.set(0.0);
                this.bottomGripper.set(0.0);
                terminate = true;
           }
        }
    }
    
    public void terminate(){
        this.terminate = true;
    }
}