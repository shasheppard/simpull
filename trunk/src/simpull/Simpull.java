/*
    Copyright (c) 2008, Shaun Curtis Sheppard
    All rights reserved.
    
    Redistribution and use in source and binary forms, with or without 
    modification, are permitted provided that the following conditions are met:

        * Redistributions of source code must retain the above copyright 
          notice, this list of conditions and the following disclaimer.
        * Redistributions in binary form must reproduce the above copyright 
          notice, this list of conditions and the following disclaimer in the 
          documentation and/or other materials provided with the distribution.
        * Neither the name of Shaun Curtis Sheppard nor the names of its 
          contributors may be used to endorse or promote products derived from 
          this software without specific prior written permission.
    
    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
    IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
    ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
    LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
    CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
    SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
    INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
    CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
    ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
    POSSIBILITY OF SUCH DAMAGE.
*/

package simpull;
	
import java.util.ArrayList;
import java.util.List;

import simpull.events.EventManager;

/**
 * The main engine class of simpull. 
 */
public final class Simpull {
	
	/**
	 * The global damping. Values should be between 0 and 1. Higher numbers result
	 * in less damping. A value of 1 is no damping. A value of 0 won't allow any
	 * particles to move. The default is 1.
	 * 
	 * Damping will slow down the simulation and make it more stable. If the simulation
	 * is becoming very unstable, try applying more damping. 
	 * 
	 * The damping value. Values should be >=0 and <=1.
	 */
	public static float damping;
	
	static List<IForce> forces;
		
	private static List<Group> groups;
	private static float timeStep;
	
	private static int constraintCycles;
	private static int constraintCollisionCycles;
	
	private static int iTmp;
	
	/**
	 * Initializes the engine. This must be called prior to adding any 
	 * particles or constraints.
	 * 
	 * @param engineTimeStep The delta time value for the engine. This parameter can be used
	 * in conjunction with speed at which {@link Simpull#step()} is called 
	 * to change the speed of the simulation. Typical values are 1/3 or 1/4. Lower 
	 * values result in slower, but more accurate simulations, and higher ones 
	 * result in faster, less accurate ones. Note that this only applies to the 
	 * forces added to particles. If you do not add any forces, the engineTimeStep will not
	 * matter.
	 */
	public static void init(float engineTimeStep/*= 0.25*/) {
		timeStep = engineTimeStep * engineTimeStep;
		
		groups = new ArrayList<Group>();
		forces = new ArrayList<IForce>();
		
		damping = 1;
		constraintCycles = 0;
		constraintCollisionCycles = 1;
	}

	public static int getConstraintCycles() {
		return constraintCycles;
	}
	
	/**
	 * Dictates the number of times in a single {@link Simpull#step()} cycle that 
	 * the constraints have their positions corrected. Increasing this number can result in
	 * stiffer, more stable configurations of constraints, especially when they are in large
	 * complex arrangements. Higher values equal less performance (more calculations).
	 *
	 * This setting differs from the constraintCollisionCycles property in that it
	 * only resolves constraints during {@link Simpull#step()}. The default value
	 * is 0. Because this property does not correct for collisions, only use it when
	 * the collisions of an arrangement of particles and constraints are not an issue. If 
	 * set higher than the default of 0, then constraintCollisionCycles
	 * should at least be 1, in order to check collisions one time during {@link Simpull#step()}.
	 * 
	 * @param engineConstraintCycles the number of cycles each step where the constraints will be resolved.
	 */
	public static void setConstraintCycles(int engineConstraintCycles) {
		constraintCycles = engineConstraintCycles;
	}	
	
	public static int getConstraintCollisionCycles() {
		return constraintCollisionCycles;
	}
	
	/**
	 * Dictates the number of times in a single {@link Simpull#step()} cycle that
	 * the constraints have their positions corrected and particles in collision have their
	 * positions corrected. This can greatly increase stability and prevent breakthroughs,
	 * especially with large complex arrangements of constraints and particles. Higher values 
	 * equal less performance (more calculations).
	 *
	 * This setting differs from the constraintCycles property in that it
	 * resolves both constraints and collisions during a {@link Simpull#step()},
	 * as opposed to just the constraints. The default value is 1.
	 * 
	 * @param engineConstraintCollisionCycles
	 */
	public static void setConstraintCollisionCycles(int engineConstraintCollisionCycles) {
		constraintCollisionCycles = engineConstraintCollisionCycles;
	}			
	
	/**
	 * @param force an {@link IForce} to apply to all particles in the system.
	 * Once a force is added it is continually applied each
	 * Simpull.step() cycle.
	 */ 
	public static void add(IForce force) {
		forces.add(force);
	}
	
	/** @param force an {@link IForce} to remove from the engine. */
	public static void remove(IForce force) {
		forces.remove(force);
	}
	
	/** Removes all forces from the engine. */
	public static void removeAllForces() {
		forces.clear();
	}			
	
	/** @param newGroups {@link Group} instances to add to the simpull system. */
	public static void add(Group... newGroups) {
		for (Group group : newGroups) {
			groups.add(group);
			group.setIsParented(true);
			group.init();
		}
	}
	
	/** @param group a {@link Group} to remove from the simpull system. */
	public static void remove(Group group) {
		groups.remove(group);
		group.setIsParented(false);
		group.cleanup();
	}
	
	/**
	 * The main step of the simpull engine. This method should be called
	 * continuously to advance the simulation. The faster this method is 
	 * called, the faster the simulation will run. Usually you would call
	 * this in your main program loop. 
	 */			
	public static void step() {
		// Perform the integration step
		for (Group group : groups) {
			group.integrate(timeStep);
		}

		for (iTmp = 0; iTmp < constraintCycles; ++iTmp) {
			for (Group group : groups) {
				group.satisfyConstraints();
			}
		}
		
		for (iTmp = 0; iTmp < constraintCollisionCycles; ++iTmp) {
			for (Group group : groups) {
				group.satisfyConstraints();
			}
			// Perform collision detection
			for (Group group : groups) {
				group.checkCollisions();
			}
		}
		
		EventManager.manageEvents();
	}

}