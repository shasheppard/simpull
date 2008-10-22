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
	
public class Rectangle extends Particle {

	int[] extents;
	Vector2f[] axes;
	
	private static final int defaultMass = FP.ONE; // FIXME this default is unqualified
	
	/**
	 * @param x The initial x position.
	 * @param y The initial y position.
	 * @param width The width of this particle.
	 * @param height The height of this particle.
	 * @param rotation The rotation of this particle in radians.
	 * @param isFixed Determines if the particle is fixed or not. Fixed particles
	 * are not affected by forces or collisions and are good to use as surfaces.
	 * Non-fixed particles move freely in response to collision and forces.
	 * @param mass The mass of the particle
	 * @param elasticity The elasticity of the particle. Higher values mean more elasticity.
	 * @param friction The surface friction of the particle. 

	 * Note that a {@link Rectangle} can be fixed and still have the rotation property 
	 * changed.
	 */
	public Rectangle (
			int x, 
			int y, 
			int width, 
			int height, 
			int rotation/*= 0*/, 
			boolean isFixed/*= false*/,
			int mass/*= 1*/, 
			int elasticity/*= 0.3*/,
			int friction/*= 0*/) {
		super(x, y, isFixed, mass, elasticity, friction);
		
		extents = new int[] {width >> 1, height >> 1};
		
		axes = new Vector2f[] {new Vector2f(0,0), new Vector2f(0,0)};
		setRotation(rotation);
	}

	/** Convenience constructor to use the default values for what is not passed. */
	public Rectangle (
			int x, 
			int y, 
			int width, 
			int height, 
			int rotation/*= 0*/, 
			boolean isFixed/*= false*/) {
		this(x,y,width,height,rotation,isFixed,defaultMass,FP.from(0.3f),0);
	}
	
	@Override
	public void setRotation(int rotation) {
		super.setRotation(rotation);
		setAxes(rotation);
	}
		
	/**
	 * Sets up the visual representation of this RectangleParticle. This method is called 
	 * automatically when an instance of this RectangleParticle's parent Group is added to 
	 * the Simpull, when  this RectangleParticle's Composite is added to a Group, or the 
	 * RectangleParticle is added to a Composite or Group.
	 */
	@Override
	public void init() {
		cleanup();
	}
	
	public void setWidth(int width) {
		extents[0] = width >> 1;
	}

	public int getWidth() {
		return (int) (((long) extents[0] * FP.TWO) >> FP.FRACTION_BITS);
		// above replaces return extents[0] * 2;
	}

	public void setHeight(int height) {
		extents[1] = height >> 1;
	}

	public int getHeight() {
		return (int) (((long) extents[1] * FP.TWO) >> FP.FRACTION_BITS);
		// above replaces return extents[1] * 2;
	}
			
	Interval getProjection(Vector2f axis) {
		int radius = 
			(int) (((long) extents[0] * (FP.abs((int) (((long) axis.x * axes[0].x) >> FP.FRACTION_BITS) + (int) (((long) axis.y * axes[0].y) >> FP.FRACTION_BITS)))) >> FP.FRACTION_BITS) + 
			(int) (((long) extents[1] * (FP.abs((int) (((long) axis.x * axes[1].x) >> FP.FRACTION_BITS) + (int) (((long) axis.y * axes[1].y) >> FP.FRACTION_BITS)))) >> FP.FRACTION_BITS);
// above replaces		float radius =
//			extents[0] * Math.abs(axis.x * axes[0].x + axis.y * axes[0].y)+
//			extents[1] * Math.abs(axis.x * axes[1].x + axis.y * axes[1].y);
		
		int c = (int) (((long) samp.x * axis.x) >> FP.FRACTION_BITS) + (int) (((long) samp.y * axis.y) >> FP.FRACTION_BITS);
		// above replaces float c = samp.x * axis.x + samp.y * axis.y;
		
		interval.min = c - radius;
		interval.max = c + radius;
		return interval;
	}

	private void setAxes(int rotation) {
		int sin = FP.sin(rotation);
		int cos = FP.cos(rotation);
		
		axes[0].x = cos;
		axes[0].y = sin;
		axes[1].x = -sin;
		axes[1].y = cos;
	}

}