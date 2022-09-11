import { Matrix3, Object3D, Vector3, Euler } from 'three';

class URDFCollider extends Object3D {

	constructor( ...args ) {

		super( ...args );
		this.isURDFCollider = true;
		this.type = 'URDFCollider';

	}

}

class URDFVisual extends Object3D {

	constructor( ...args ) {

		super( ...args );
		this.isURDFVisual = true;
		this.type = 'URDFVisual';

	}

}

class URDFInertialFrame {

	constructor() {

		this.position = new Vector3();
		this.rotation = new Euler();
		this.mass = 0;
		this.inertia = new Matrix3();

	}

	copy( source ) {

		this.position.copy( source.position );
		this.rotation.copy( source.rotation );
		this.inertia.copy( source.inertia );
		this.mass = source.mass;

	}

}

class URDFLink extends Object3D {

	constructor( ...args ) {

		super( ...args );
		this.isURDFLink = true;
		this.type = 'URDFLink';

		this.inertial = new URDFInertialFrame();

	}

	copy( source, recursive ) {

		super.copy( source, recursive );

		this.inertial.copy( source.inertial );

	}

}

class URDFLimit {

	constructor() {

		this.lower = 0;
		this.upper = 0;
		this.effort = 0;
		this.velocity = 0;

	}

	copy( source ) {

		this.lower = source.lower;
		this.upper = source.upper;
		this.effort = source.effort;
		this.velocity = source.velocity;

	}

}

class URDFJoint extends Object3D {

	constructor( ...args ) {

		super( ...args );

		this.isURDFJoint = true;
		this.type = 'URDFJoint';

		this.jointType = 'fixed';
		this.axis = new Vector3( 1, 0, 0 );
		this.limit = new URDFLimit();

	}

	copy( source, recursive ) {

		super.copy( source, recursive );

		this.jointType = source.jointType;
		this.axis.copy( source.axis );
		this.limit.copy( source.limit );

	}

}

class URDFMimicJoint extends URDFJoint {

	constructor( ...args ) {

		super( ...args );
		this.type = 'URDFMimicJoint';
		this.mimicJoint = null;
		this.offset = 0;
		this.multiplier = 1;

	}

}

class URDFRobot extends URDFLink {

	constructor( ...args ) {

		super( ...args );
		this.isURDFRobot = true;

		this.robotName = '';

	}

	copy( source, recursive ) {

		super.copy( source, recursive );

		this.robotName = source.robotName;

	}

}

export { URDFRobot, URDFLink, URDFJoint, URDFMimicJoint, URDFVisual, URDFCollider };
