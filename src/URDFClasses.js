import { Matrix3, Object3D, Vector3 } from 'three';

class URDFBase extends Object3D {

	constructor( ...args ) {

		super( ...args );
		this.urdfNode = null;
		this.urdfName = '';

	}

	copy( source, recursive ) {

		super.copy( source, recursive );

		this.urdfNode = source.urdfNode;
		this.urdfName = source.urdfName;

		return this;

	}

}

class URDFCollider extends URDFBase {

	constructor( ...args ) {

		super( ...args );
		this.isURDFCollider = true;
		this.type = 'URDFCollider';

	}

}

class URDFVisual extends URDFBase {

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
		this.inertial = new Matrix3();

	}

}

class URDFLink extends URDFBase {

	constructor( ...args ) {

		super( ...args );
		this.isURDFLink = true;
		this.type = 'URDFLink';

		this.inertial = new URDFInertialFrame();

	}

}

class URDFJoint extends URDFBase {

	get jointType() {

		return this._jointType;

	}

	set jointType( v ) {

		if ( this.jointType === v ) return;
		this._jointType = v;
		this.matrixWorldNeedsUpdate = true;
		switch ( v ) {

			case 'fixed':
				this.jointValue = [];
				break;

			case 'continuous':
			case 'revolute':
			case 'prismatic':
				this.jointValue = new Array( 1 ).fill( 0 );
				break;

			case 'planar':
				this.jointValue = new Array( 2 ).fill( 0 );
				break;

			case 'floating':
				this.jointValue = new Array( 6 ).fill( 0 );
				break;

		}

	}

	get angle() {

		return this.jointValue[ 0 ];

	}

	constructor( ...args ) {

		super( ...args );

		this.isURDFJoint = true;
		this.type = 'URDFJoint';

		this.jointType = 'fixed';
		this.axis = new Vector3( 1, 0, 0 );
		this.limit = { lower: 0, upper: 0 };

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

		this.robotName = null;

	}

}

export { URDFRobot, URDFLink, URDFJoint, URDFMimicJoint, URDFVisual, URDFCollider };
