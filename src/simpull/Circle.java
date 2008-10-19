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
	
public class Circle extends Particle {

	private int radius;
	
	/**
	 * @param x The initial x position of this particle.
	 * @param y The initial y position of this particle.
	 * @param radius The radius of this particle.
	 * @param isFixed Determines if the particle is fixed or not. Fixed particles
	 * are not affected by forces or collisions and are good to use as surfaces.
	 * Non-fixed particles move freely in response to collision and forces.
	 * @param mass The mass of the particle.
	 * @param elasticity The elasticity of the particle. Higher values mean more elasticity or 'bounciness'.
	 * @param friction The surface friction of the particle.
	 */
	public Circle (
			int x, 
			int y, 
			int radius, 
			boolean isFixed/*= false*/,
			int mass/*= 1*/, 
			int elasticity/* = 0.3*/,
			int friction/*= 0*/) {
		super(x, y, isFixed, mass, elasticity, friction);
		this.radius = radius;
	}

	/** @return radius of the particle. */
	public int getRadius() {
		return radius;
	}		
	
	public void setRadius(int radius) {
		this.radius = radius;
	}
	
	/**
	 * Sets up the visual representation of this CircleParticle. This method is called 
	 * automatically when an instance of this CircleParticle's parent Group is added to 
	 * the Simpull, when  this CircleParticle's Composite is added to a Group, or the 
	 * CircleParticle is added to a Composite or Group.
	 */		
	@Override
	public void init() {
		cleanup();
	}
	
	Interval getProjection(Vector2f axis) {
		int c = (int) (((long) samp.x * axis.x) >> FP.FRACTION_BITS) + (int) (((long) samp.y * axis.y) >> FP.FRACTION_BITS);
		// above replaces float c = samp.x * axis.x + samp.y * axis.y;
		interval.min = c - radius;
		interval.max = c + radius;
		
		return interval;
	}
	
	Interval getIntervalX() {
		interval.min = samp.x - radius;
		interval.max = samp.x + radius;
		return interval;
	}
	
	Interval getIntervalY() {
		interval.min = samp.y - radius;
		interval.max = samp.y + radius;
		return interval;
	}
	
}	