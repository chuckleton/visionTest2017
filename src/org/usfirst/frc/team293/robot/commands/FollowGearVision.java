package org.usfirst.frc.team293.robot.commands;

import org.usfirst.frc.team293.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class FollowGearVision extends Command {
	private static long lastTime = System.currentTimeMillis();
	private static final int Dt = 50;
	private static boolean lost = false;
	private static long timeLost = 0;
	private static final int timeToLost = 1600;
	
	
    	public FollowGearVision(){
    		requires(Robot.Camera);
    	}
    	
    	// Called just before this Command runs the first time
        protected void initialize() {
        }

        // Called repeatedly when this Command is scheduled to run
        protected void execute() {
        	int newData = Robot.Camera.getPiData();
        	if(newData == -1 && !lost){
        		lost = true;
        		timeLost = System.currentTimeMillis();
        	}
        	if(newData == 1 || (System.currentTimeMillis() - lastTime > Dt && !lost)){
        		Robot.Camera.updatePID(newData);
        		Robot.Camera.setServos();
        		//SmartDashboard.putNumber("Azimuth", Robot.Camera.getAzimuth());
            	//SmartDashboard.putNumber("Distance", Robot.Camera.getDistance());
            	lost = false;
        		lastTime = System.currentTimeMillis();
        	}else if(needToSearch()){
        		Robot.Camera.search();
        		Robot.Camera.setServos();
        		lastTime = System.currentTimeMillis();
        	}
        }
        
        private boolean needToSearch(){
        	return (lost && System.currentTimeMillis() - lastTime > Dt 
        			&& System.currentTimeMillis() - timeLost > timeToLost);
        }
    	
    	protected boolean isFinished() {
            return false;
        }
    	
    	// Called once after isFinished returns true
        protected void end() {
        }

        // Called when another command which requires one or more of the same
        // subsystems is scheduled to run
        protected void interrupted() {
        }
}
