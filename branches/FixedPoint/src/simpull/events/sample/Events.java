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

package simpull.events.sample;
	
import simpull.Collision;
import simpull.Particle;
import simpull.Vector2f;
import simpull.events.Event;

/** 
 * Events used explicitly in Simpull.  
 * Use this method as a template when/if you want to add more events.
 * Notitce the XXXEvent constructor accepts the same parameters as the
 * matching XXXData constructor and how the two are related and used.
 */
public final class Events {

	/**
	 * CollisionEvent objects are dispatched during a collision between two 
	 * {@link Particle} instances.
	 */
	public static final class CollisionEvent extends Event<Collision> {
		public CollisionEvent(Particle me, Particle you, 
				Vector2f vNormal, Vector2f vTangent) {
			super(new Collision(me, you, vNormal, vTangent));	
		}
		public CollisionEvent(Collision collision) {
			super(collision);
		}
	}
	
	public static final class FirstCollisionEvent extends Event<Collision> {
		public FirstCollisionEvent(Particle me, Particle you, 
				Vector2f vNormal, Vector2f vTangent) {
			super(new Collision(me, you, vNormal, vTangent));	
		}
		public FirstCollisionEvent(Collision collision) {
			super(collision);
		}
	}
	
	public static final class AngleJointBreakEvent extends Event<AngleJointBreakData> {
		public AngleJointBreakEvent(float radiansBrokenBy) {
			super(new AngleJointBreakData(radiansBrokenBy));
		}
	}
	
	public static final class AngleJointBreakData {
		public float radiansBrokenBy;
		public AngleJointBreakData(float radiansBrokenBy) {
			this.radiansBrokenBy = radiansBrokenBy;
		}
	}
	
}