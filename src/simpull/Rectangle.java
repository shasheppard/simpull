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
	
public strictfp class Rectangle extends Particle {

	private float[] extents;
	private Vector2f[] axes;
	
	private static final float defaultMass = 0.6f; // FIXME this default is unqualified
	
	/**
	 * @param x The initial x position.
	 * @param y The initial y position.
	 * @param width The width of this particle.
	 * @param height The height of this particle.
	 * @param rotation The rotation of this particle in radians.
	 * @param fixed Determines if the particle is fixed or not. Fixed particles
	 * are not affected by forces or collisions and are good to use as surfaces.
	 * Non-fixed particles move freely in response to collision and forces.
	 * @param mass The mass of the particle
	 * @param elasticity The elasticity of the particle. Higher values mean more elasticity.
	 * @param friction The surface friction of the particle. 
	 * <p>
	 * Note that RectangleParticles can be fixed but still have their rotation property 
	 * changed.
	 * </p>
	 */
	public Rectangle (
			float x, 
			float y, 
			float width, 
			float height, 
			float rotation/*= 0*/, 
			boolean fixed/*= false*/,
			float mass/*= 1*/, 
			float elasticity/*= 0.3*/,
			float friction/*= 0*/) {
		
		super(x, y, fixed, mass, elasticity, friction);
		
		extents = new float[] {width/2, height/2};
		axes = new Vector2f[] {new Vector2f(0,0), new Vector2f(0,0)};
		setRotation(rotation);
	}

	/** Convenience constructor to use the default values for what is not passed. */
	public Rectangle (
			float x, 
			float y, 
			float width, 
			float height, 
			float rotation/*= 0*/, 
			boolean fixed/*= false*/) {
		this(x,y,width,height,rotation,fixed,defaultMass,0.3f,0f);
	}
	
	@Override
	public void setRotation(float rotation) {
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
	
	public void setWidth(float width) {
		extents[0] = width/2;
	}

	public float getWidth() {
		return extents[0] * 2;
	}

	public void setHeight(float height) {
		extents[1] = height / 2;
	}

	public float getHeight() {
		return extents[1] * 2;
	}
			
	Vector2f[] getAxes() {
		return axes;
	}
	
	float[] getExtents() {
		return extents;
	}
	
	Interval getProjection(Vector2f axis) {
		float radius =
			extents[0] * Math.abs(axis.x * axes[0].x + axis.y * axes[0].y)+
			extents[1] * Math.abs(axis.x * axes[1].x + axis.y * axes[1].y);
		float c = samp.x * axis.x + samp.y * axis.y;
		interval.min = c - radius;
		interval.max = c + radius;
		return interval;
	}

	private void setAxes(float rotation) {
		float s = (float)Math.sin(rotation);
		float c = (float)Math.cos(rotation);
		
		axes[0].x = c;
		axes[0].y = s;
		axes[1].x = -s;
		axes[1].y = c;
	}

}