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
	
/**
 * The Composite class can contain Particles, and Constraints.
 * The most common use is to create a complex physics object.
 * The Composite can be rotated all as one around a single point. 
 * Composites can be added to a parent Group, along with Particles and Constraints.  
 * Members of a Composite are not checked for collision with one another, internally.
 */ 
public strictfp class Composite extends BaseCollection implements IPhysicsObject {
	
	private Vector2f delta = new Vector2f();
	
	/**
	 * Rotates the Composite to an angle specified in radians, around a given center
	 */
	public void rotateByRadian(float angleRadians, Vector2f center) {
		for (Particle particle : getParticles()) {
			Vector2f diff = particle.getCenter();
			diff.x -= center.x;
			diff.y -= center.y;
			float radius = (float)Math.sqrt(diff.x * diff.x + diff.y * diff.y);
			float angle = getRelativeAngle(center, particle.getCenter()) + angleRadians;
			particle.setX((float)(Math.cos(angle) * radius) + center.x);
			particle.setY((float)(Math.sin(angle) * radius) + center.y);
		}
	}  
	
	/** @return the fixed state of the Composite. */	
	public boolean getFixed() {
		for (Particle particle : getParticles()) {
			if (!particle.getFixed()) {
				return false;	
			}
		}
		return true;
	}

	/**
	 * @param isFixed the fixed state of the Composite. Setting this value to true or false will
	 * set all of this Composite's component particles to that value. Getting this 
	 * value will return false if any of the component particles are not fixed.
	 */	
	public void setFixed(boolean isFixed) {
		for (Particle particle : getParticles()) {
			particle.setFixed(isFixed);	
		}
	}
	
	private float getRelativeAngle(Vector2f center, Vector2f point) {
		delta.x = point.x - center.x;
		delta.y = point.y - center.y;
		return (float)Math.atan2(delta.y, delta.x);
	}
	
}