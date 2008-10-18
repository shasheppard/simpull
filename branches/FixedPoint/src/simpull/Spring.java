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
 * A Spring-like constraint that connects two particles
 */
public strictfp class Spring extends SimpleSpring {
	
	private float min;
	private float mid;
	private float max;
	private float force;
	
	/**
	 * 
	 * @param p1
	 * @param p2
	 * @param isCollidable
	 * @param min
	 * @param mid
	 * @param max
	 * @param force
	 */
	public Spring(
			Particle p1, 
			Particle p2,
			boolean isCollidable,
			float min, 
			float mid, 
			float max, 
			float force) {
		// FIXME: the last 3 parameters are hardcoded here for a quick dev test.
		super(p1, p2, isCollidable, 2, 1, true);
		this.min = min;
		this.mid = mid;
		this.max = max;
		this.force = force;
	}
	
	@Override
	public void resolve() {
	    // Get vector from other verlet to me
	    Vector2f diff = 
	    	new Vector2f(particle1.position.x - particle2.position.x, 
	    			particle1.position.y - particle2.position.y);
	    Vector2f mid = 
	    	new Vector2f(particle1.position.x + particle2.position.x, 
	    			particle1.position.y + particle2.position.y);
	    mid.divEquals(2.0f);

	    float radius = (float)Math.sqrt(diff.x * diff.x + diff.y * diff.y);
	    // handle case where points are the same
	    if (radius == 0.0f) {
	        diff = new Vector2f(1.0f, 0.0f);
	        radius = (float)Math.sqrt(diff.x * diff.x + diff.y * diff.y);
	    }
	    if (radius < min) {
	    	radius = min;
	    } else if (radius > max) {
	    	radius = max;
	    }

	    // Scale it to the required radius
	    Vector2f diffNormalized = diff.normalize();
	    diff.x = diffNormalized.x * radius;
	    diff.y = diffNormalized.y * radius;
	    // and set the point
    	diff.divEquals(2f);
        particle1.position.x = mid.x + diff.x; 
        particle1.position.y = mid.y + diff.y; 

        particle2.position.x = mid.x - diff.x; 
        particle2.position.y = mid.y - diff.y; 
	    
	    // The forces added here will be applied in the following APEninge.step() call.
	    IForce p1Force = getForce(particle1, particle2);
	    particle1.addForce(p1Force);
	    
	    IForce p2Force = getForce(particle2, particle1);
	    particle2.addForce(p2Force);
	}

	private VectorForce getForce(Particle particleA, Particle particleB) {
	    Vector2f diff = 
	    	new Vector2f(particleA.position.x - particleB.position.x, 
	    		particleA.position.y - particleB.position.y);
	    // handle case where points are the same
	    if (Math.sqrt(diff.x * diff.x + diff.y * diff.y) < 0.000001) {
	    	diff.x = 1f;
	    	diff.y = 0f;
	    }
	    diff.normalizeEquals();
	    Vector2f middle = new Vector2f(
	    		particleB.position.x + (diff.x * mid), 
	    		particleB.position.y + (diff.y * mid));
	    Vector2f distToMiddle = new Vector2f(
	    		middle.x - particleA.position.x,
	    		middle.y - particleA.position.y);
	    return new VectorForce(true, 
	    		distToMiddle.x * force, 
	    		distToMiddle.y * force);
	}

}