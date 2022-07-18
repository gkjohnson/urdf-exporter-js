import { Euler, SphereGeometry, BoxGeometry, CylinderGeometry } from 'three';
import {
	getChildLink,
	getParentJoint,
	getParentLink,
	getRelativeOriginNode,
	getCylinderRelativeOriginNode,
	repeatChar,
	traverseImmediateMeaningfulNodes,
	getInertiaNode,
} from './utils.js';

// http://wiki.ros.org/urdf/XML/
const _euler = new Euler();
export class URDFExporter {

	constructor() {

		this.indent = '\t';
		this.processGeometryCallback = () => {

			return null;

		};

	}

	parse( root ) {

		const { indent, processGeometryCallback } = this;

		const indent1 = repeatChar( indent, 1 );
		const indent2 = repeatChar( indent, 2 );
		const indent3 = repeatChar( indent, 3 );
		const indent4 = repeatChar( indent, 4 );

		let result = '';
		if ( ! root.isURDFRobot ) {

			console.warn( 'URDFExporter: Root link is expected to be a URDFRobot instance.' );

		}

		// initialize the names of joints and links that are unnamed
		const nameMap = new Map();
		let linkIndex = 0;
		let jointIndex = 0;
		root.traverse( c => {

			if ( c.isURDFLink ) {

				if ( c.name ) {

					nameMap.set( c, c.name );

				} else {

					nameMap.set( c, `link_${ linkIndex ++ }` );

				}

			} else if ( c.isURDFJoint ) {

				if ( c.name ) {

					nameMap.set( c, c.name );

				} else {

					nameMap.set( c, `joint_${ jointIndex ++ }` );

				}

			}

		} );


		const linkNodes = [];
		const jointNodes = [];

		processLink( root );

		result += '<?xml version="1.0"?>\n';

		result += `<robot name="${ root.robotName || 'robot' }">\n`;

		result += linkNodes.join( '\n' );

		result += '\n';

		result += jointNodes.join( '\n' );

		result += '</robot>\n';

		return result;

		function processVisualContents( node ) {

			const parentLink = getParentLink( node );
			const relativeParent = getParentJoint( node ) || parentLink;
			const children = node.children;
			let result = '';

			// if we have one child then it might be a geometric result
			if ( children.length === 1.0 && children[ 0 ].isMesh ) {

				const mesh = children[ 0 ];
				if ( mesh.geometry instanceof SphereGeometry ) {

					// sphere
					result += `${ indent3 }${ getRelativeOriginNode( mesh, relativeParent ) }`;

					const radius = mesh.geometry.parameters.radius * mesh.scale.x;
					result += `${ indent3 }<geometry>\n`;
					result += `${ indent4 }<sphere radius="${ radius }" />\n`;
					result += `${ indent3 }</geometry>\n`;

				} else if ( mesh.geometry instanceof BoxGeometry ) {

					// box
					result += `${ indent3 }${ getRelativeOriginNode( mesh, relativeParent ) }`;

					let { width, height, depth } = mesh.geometry.parameters;
					width *= mesh.scale.x;
					height *= mesh.scale.y;
					depth *= mesh.scale.z;

					result += `${ indent3 }<geometry>\n`;
					result += `${ indent4 }<box size="${ width } ${ height } ${ depth }" />\n`;
					result += `${ indent3 }</geometry>\n`;

				} else if ( mesh.geometry instanceof CylinderGeometry ) {

					// cylinder
					// TODO: include three.js rotation offset here
					result += `${ indent3 }${ getCylinderRelativeOriginNode( mesh, relativeParent ) }`;

					let { radiusTop, height } = mesh.geometry.parameters;
					radiusTop *= radiusTop * mesh.scale.x;
					height *= mesh.scale.y;

					result += `${ indent3 }<geometry>\n`;
					result += `${ indent4 }<cylinder length="${ height }" radius="${ radiusTop }" />\n`;
					result += `${ indent3 }</geometry>\n`;

				}

			}

			if ( result === '' ) {

				// mesh
				const nodeClone = node.clone();
				nodeClone.position.set( 0, 0, 0 );
				nodeClone.rotation.set( 0, 0, 0 );
				nodeClone.updateMatrixWorld();

				const path = processGeometryCallback( nodeClone, parentLink );
				if ( path !== null ) {

					result += `${ indent3 }${ getRelativeOriginNode( node, relativeParent ) }`;
					result += `${ indent3 }<geometry>\n`;
					result += `${ indent3 }<mesh filename="${ path }"/>\n`;
					result += `${ indent3 }</geometry>\n`;

				}

			}

			return result;

		}


		function processLink( link ) {

			let result = '';
			result += `${ indent1 }<link name="${ nameMap.get( link ) }">\n`;

			// inertia
			const inertial = link.inertial;
			if ( inertial ) {

				const { origin, rotation, mass, inertia } = inertial;
				_euler.copy( rotation ).reorder( 'zyx' );

				result += `${ indent2 }<inertial>\n`;
				result += `${ indent3 }<origin xyz="${ origin.x } ${ origin.y } ${ origin.z }" rpy="${ _euler.x } ${ _euler.y } ${ _euler.z }" />\n`;
				result += `${ indent3 }<mass value="${ mass }/>\n`;
				result += `${ indent3 }${ getInertiaNode( inertia ) }`;
				result += `${ indent2 }</inertial>\n`;

			}

			// process any necessary child information
			const childJoints = [];
			traverseImmediateMeaningfulNodes( link, child => {

				if ( child.isURDFJoint ) {

					childJoints.push( child );

				} else if ( child.isURDFVisual ) {

					const contents = processVisualContents( child );

					if ( contents !== '' ) {

						result += `${ indent2 }<visual`;
						if ( child.name ) {

							result += ` name="${ child.name }"`;

						}

						result += '>\n';
						result += contents;
						result += `${ indent2 }</visual>\n`;

					}

				} else if ( child.isURDFCollider ) {

					const contents = processVisualContents( child );
					if ( contents !== '' ) {

						result += `${ indent2 }<collider`;
						if ( child.name ) {

							result += ` name="${ child.name }"`;

						}

						result += '>\n';
						result += contents;
						result += `${ indent2 }</collider>\n`;

					}

				} else {

					console.warn( `URDFExporter: Link "${ link.name }" cannot contain another link.` );

				}

			} );

			result += `${ indent1 }</link>\n`;
			linkNodes.push( result );

			for ( let i = 0, l = childJoints.length; i < l; i ++ ) {

				processJoint( childJoints[ i ] );

			}

		}

		function processJoint( joint ) {

			// warn user of invalid structure
			let totalLinks = 0;
			let totalJoints = 0;
			let totalVisual = 0;
			let totalCollider = 0;
			traverseImmediateMeaningfulNodes( joint, child => {

				if ( child.isURDFJoint ) {

					totalJoints ++;

				} else if ( child.isURDFVisual ) {

					totalVisual ++;

				} else if ( child.isURDFCollider ) {

					totalCollider ++;

				} else if ( child.isURDFLink ) {

					totalLinks ++;

				}

			} );

			if ( totalLinks > 1 ) console.warn( `URDFExporter: too many links are children of Joint ${ joint.name }.` );
			if ( totalJoints > 0 ) console.warn( `URDFExporter: joints cannot be children of Joint ${ joint.name }.` );
			if ( totalVisual > 0 ) console.warn( `URDFExporter: visual nodes cannot be children of Joint ${ joint.name }.` );
			if ( totalCollider > 0 ) console.warn( `URDFExporter: collider nodes cannot be children of Joint ${ joint.name }.` );

			// find the relevant parent and child joints
			const parentLink = getParentLink( joint );
			const parentJoint = getParentJoint( joint );
			const childLink = getChildLink( joint );

			// construct the node
			let result = '';
			result += `${ indent1 }<joint name="${ nameMap.get( joint ) }" type="${ joint.jointType || 'fixed' }">\n`;

			result += `${ indent2 }${ getRelativeOriginNode( joint, parentJoint || parentLink ) }`;

			result += `${ indent2 }<parent link="${ parentLink.name }"/>\n`;

			result += `${ indent2 }<child link="${ childLink.name }"/>\n`;

			if ( joint.jointType === 'revolute' || joint.jointType === 'continuous' || joint.jointType === 'prismatic' ) {

				const axis = joint.axis;
				result += `${ indent2 }<axis xyz="${ axis.x } ${ axis.y } ${ axis.z }"/>\n`;

			}

			if ( joint.limit && joint.jointType !== 'fixed' ) {

				const limit = joint.limit;
				result += `${ indent2 }<limit effort="${ limit.effort || 0 }" velocity="${ limit.velocity || 0 }"`;
				if ( joint.jointType !== 'continuous' && 'lower' in limit && 'upper' in limit ) {

					result += ` lower="${ limit.lower }" upper="${ limit.upper }"`;

				}

				result += '/>\n';

			}

			result += `${ indent1 }</joint>\n`;

			jointNodes.push( result );

			processLink( childLink );

		}

	}

}
