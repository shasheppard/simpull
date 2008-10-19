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
		
/** A force represented by a 2D vector. */
public class VectorForce implements IForce {

	private Vector2f force;	
	private boolean useMass;
	
	public VectorForce(boolean useMass, int forceX, int forceY) {
		this.useMass = useMass;
		force = new Vector2f(forceX, forceY);
	}
	
	public void setVx(int x) {
		force.x = x;
	}
	
	public void setVy(int y) {
		force.y = y;
	}
	
	public void setUseMass(boolean useMass) {
		this.useMass = useMass;
	}
	
	public Vector2f getValue(int invmass) {
		if (useMass) { 
			force.x = (int) (((long) force.x * invmass) >> FP.FRACTION_BITS);
			// above replaces force.x *= invmass;
			
			force.y = (int) (((long) force.y * invmass) >> FP.FRACTION_BITS);
			// above replaces force.y *= invmass;
		}
		return force;
	}
}