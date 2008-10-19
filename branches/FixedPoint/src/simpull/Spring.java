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
	
/** A Spring-like constraint that connects two particles */
public class Spring extends SimpleSpring {
	
	private int min;
	private int mid;
	private int max;
	private int force;
	
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
			int min, 
			int mid, 
			int max, 
			int force) {
		// FIXME: the last 3 parameters are hardcoded here for a quick dev test.
		super(p1, p2, isCollidable, FP.TWO, FP.ONE, true);
		this.min = min;
		this.mid = mid;
		this.max = max;
		this.force = force;
	}
	
	@Override
	public void resolve() {
	    // Get vector from other verlet to me
	    Vector2f diff = new Vector2f(particle1.position.x - particle2.position.x, 
    		particle1.position.y - particle2.position.y);
	    Vector2f mid = 
	    	new Vector2f(particle1.position.x + particle2.position.x, 
	    			particle1.position.y + particle2.position.y);
	    mid.divEquals(FP.TWO);

	    int radius = FP.sqrt((int) (((long) diff.x * diff.x) >> FP.FRACTION_BITS) + (int) (((long) diff.y * diff.y) >> FP.FRACTION_BITS));
	    // above replaces float radius = (float)Math.sqrt(diff.x * diff.x + diff.y * diff.y);
	    
	    // handle case where points are the same
	    if (radius == 0) {
	        diff.x = FP.ONE;
	        diff.y = 0;

		    radius = FP.ONE; // the original calculation was completely useless, we know the radius is 1
		    // above replaces radius = (float)Math.sqrt(diff.x * diff.x + diff.y * diff.y);
	    }
	    if (radius < min) {
	    	radius = min;
	    } else if (radius > max) {
	    	radius = max;
	    }

	    // Scale it to the required radius
	    Vector2f diffNormalized = diff.normalize();
	    
	    diff.x = (int) (((long) diffNormalized.x * radius) >> FP.FRACTION_BITS);
	    // above replaces diff.x = diffNormalized.x * radius;
	    
	    diff.y = (int) (((long) diffNormalized.y * radius) >> FP.FRACTION_BITS);
	    // above replaces diff.y = diffNormalized.y * radius;
	    
	    // and set the point
    	diff.divEquals(FP.TWO);
        particle1.position.x = mid.x + diff.x; 
        particle1.position.y = mid.y + diff.y; 

        particle2.position.x = mid.x - diff.x; 
        particle2.position.y = mid.y - diff.y; 
	    
	    // The forces added here will be applied in the following Simpull.step() call.
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
	    // FIXME this is horrible on performance and there is a quick/dirty way of determining this
	    int radius = FP.sqrt((int) (((long) diff.x * diff.x) >> FP.FRACTION_BITS) + (int) (((long) diff.y * diff.y) >> FP.FRACTION_BITS));
	    if (radius < FP.POINT_000001) {
	    	diff.x = FP.ONE;
	    	diff.y = 0;
	    }
	    diff.normalizeEquals();
	    
	    Vector2f middle = new Vector2f(
	    		particleB.position.x + ((int) (((long) diff.x * mid) >> FP.FRACTION_BITS)),
	    		particleB.position.y + ((int) (((long) diff.y * mid) >> FP.FRACTION_BITS)));
// above replaces	    Vector2f middle = new Vector2f(
//	    		particleB.position.x + (diff.x * mid), 
//	    		particleB.position.y + (diff.y * mid));
	    
	    Vector2f distToMiddle = new Vector2f(
	    		middle.x - particleA.position.x,
	    		middle.y - particleA.position.y);
	    
	    return new VectorForce(true, 
	    		(int) (((long) distToMiddle.x * force) >> FP.FRACTION_BITS),
	    		(int) (((long) distToMiddle.y * force) >> FP.FRACTION_BITS));
// above replaces	    return new VectorForce(true, 
//	    		distToMiddle.x * force, 
//	    		distToMiddle.y * force);
	}

}