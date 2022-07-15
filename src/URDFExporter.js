import { Euler, Matrix4, Quaternion, Vector3 } from 'three';

// http://wiki.ros.org/urdf/XML/
const _euler = new Euler();
const _quaternion = new Quaternion();
const _scale = new Vector3();
const _position = new Vector3();
const _matrix = new Matrix4();

function repeatChar(char, count) {

    let result = '';
    for ( let i = 0; i < count; i ++ ) {

        result += char;

    }

    return result;

}

// returns the transform of root relative to parent
function getRelativeOriginNode(root, parent) {

    _matrix.copy(parent.matrixWorld).invert().premultiply(root.matrixWorld);
    return getOriginNode(_matrix);

}

// returns the origin node for the given matrix
function getOriginNode(matrix) {

    matrix.decompose(_position, _quaternion, _scale);
    _euler.setFromQuaternion(_quaternion, 'zyx');

    return `<origin xyz="${ _position.x } ${ _position.y } ${ _position.z }" rpy="${ _euler.x } ${ _euler.y } ${ _euler.z }"/>`;

}

// recursively traverses the root until reaching a URDF node. Stops if the callback returns "true".
function traverseImmediateMeaningfulNodes(root, cb) {

    const children = root.children;
    for ( let i = 0, l = children.length; i < l; i ++) {

        const c = children[ i ];
        if (
            c.isURDFLink ||
            c.isURDFJoint ||
            c.isURDFCollider ||
            c.isURDFVisual
        ) {

            if ( cb( c ) === true ) {

                break;

            }

        }

    }

}

// recursively traverse the parent until a callback returns "true" or the robot root is found.
function traverseParents(root, cb) {

    let curr = root.parent;
    while ( curr ) {

        if ( cb( curr ) === true ) {

            break;

        }

        if ( curr.isURDFRobot ) {

            break;

        }

        curr = curr.parent;

    }

}

// returns the first found child link.
function getChildLink( root ) {

    let result = null;
    traverseImmediateMeaningfulNodes( root, node => {

        if ( node.isURDFLink ) {

            result = node;
            return true;

        }

    } );

    return result;

}

// returns the first found parent link.
function getParentLink( root ) {

    let result = null;
    traverseParents( root, node => {

        if ( node.isURDFLink ) {

            result = node;
            return true;

        }

    } );

    return result;

}

// returns the first found parent joint.
function getParentJoint( root ) {

    let result = null;
    traverseParents( root, node => {

        if ( node.isURDFJoint ) {

            result = node;
            return true;

        }

    } );

    return result;

}

export class URDFExporter {

    constructor() {

        this.indent = '\t';
        this.processGeometryCallback = () => {

            return null;

        };

    }

    parse(root) {

        const { indent, processGeometryCallback } = this;
        const indent1 = repeatChar( indent, 1 );
        const indent2 = repeatChar( indent, 2 );
        const indent3 = repeatChar( indent, 3 );
        const indent4 = repeatChar( indent, 4 );

        let result = '';
        if (!root.isURDFRobot) {

            console.warn('URDFExporter: Root link is expected to be a URDFRobot instance.');

        }

        const linkNodes = [];
        const jointNodes = [];

        processLink(root);

        result += `<robot name="${ root.robotName || 'robot' }>`;

        result += linkNodes.join('');

        result += jointNodes.join('');

        result += '</robot>'

        return result;

        function processVisualContents( node ) {

            const parentJoint = getParentJoint( node );
            const children = node.children;
            let result = '';
            result += `${ indent4 }${ getRelativeOriginNode( node, parentJoint ) }`;

            // if we have one child then it might be a geometric result
            if ( children.length === 1.0 && children[ 0 ].isMesh ) {

                const mesh = children[ 0 ];
                if ( mesh.geometry.isSphereGeometry ) {

                    // TODO
                    const radius = mesh.geometry.parameters.radius * mesh.scale.x;

                } else if ( mesh.geometry.isBoxGeometry ) {

                    // TODO
                    let { width, height, depth } = mesh.geometry.parameters;
                    width *= mesh.scale.x;
                    height *= mesh.scale.y;
                    depth *= mesh.scale.z;

                } else if ( mesh.geometry.isCylinderGeometry ) {

                    // TODO
                    let { radiusTop, height } = mesh.geometry.parameters;
                    const radius = radiusTop * mesh.scale.x;
                    height *= mesh.scale.y;

                    // TODO: include three.js rotation offset here

                } else {

                    const path = processGeometryCallback( node );
                    if ( path !== null ) {

                        result += `${ indent4 }<geometry><mesh filename="${ path }"/></geometry>`;

                    }

                }

            } else {

                const path = processGeometryCallback( node );
                if ( path !== null ) {

                    result += `${ indent4 }<geometry><mesh filename="${ path }"/></geometry>`;

                }

            }

            return result;

        }


        function processLink( link ) {

            let result = '';
            result += `${ indent1 }<link name="${ link.name }">`;

            // TODO: include inertial

            // process any necessary child information
            traverseImmediateMeaningfulNodes( link, child => {

                if ( child.isURDFJoint ) {

                    processJoint( child );

                } else if ( child.isURDFVisual ) {

                    result += `${ indent3 }<visual>`;
                    result += processVisualContents(child);
                    result += `${ indent3 }<visual>`;

                } else if ( child.isURDFCollider ) {

                    result += `${ indent3 }<collider>`;
                    result += processVisualContents(child);
                    result += `${ indent3 }<collider>`;

                } else {

                    // ???
                    console.warn();

                }

            } );

            result += `${ indent1 }</link>`;
            linkNodes.push( result );

        }

        function processJoint( joint ) {

            // warn user of invalid structure
            let totalLinks = 0;
            let totalJoints = 0;
            let totalVisual = 0;
            let totalCollider = 0;
            traverseImmediateMeaningfulNodes(joint, node => {

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

            if ( totalLinks > 1 ) console.warn(`URDFExporter: too many links are children of Joint ${ joint.name }.`);
            if ( totalJoints > 0 ) console.warn(`URDFExporter: joints cannot be children of Joint ${ joint.name }.`);
            if ( totalVisual > 0 ) console.warn(`URDFExporter: visual nodes cannot be children of Joint ${ joint.name }.`);
            if ( totalCollider > 0 ) console.warn(`URDFExporter: collider nodes cannot be children of Joint ${ joint.name }.`);

            // find the relevant parent and child joints
            const parentLink = getParentLink( joint );
            const parentJoint = getParentJoint( joint );
            const childLink = getChildLink( joint );

            // construct teh node
            let result = '';
            result += `${ indent1 }<joint name="${ joint.name }" type="${ joint.jointType || 'fixed' }">`;

            result += `${ indent2 }${ getRelativeOriginNode(joint, parentJoint) }`;

            result += `${ indent2 }<parent link="${ parentLink.name }"/>`;

            result += `${ indent2 }<child link="${ childLink.name }"/>`;

            if ( joint.jointType === 'revolute' || joint.jointType === 'continuous' || joint.jointType === 'prismatic' ) {

                const axis = joint.axis;
                result += `${ indent2 }<axis xyz="${ axis.x } ${ axis.y } ${ axis.z }"/>`;

            }

            // TODO: include more limits?
            if ( joint.limits ) {

                const limits = joint.limits;
                result += `${ indent2 }<limit effort="${ limits.effort || 0 }" velocity="${ limits.velocity }"`;
                if ( joint.jointType !== 'continuous' && 'lower' in limits && 'upper' in limits ) {

                    result += `lower="${ limits.lower }" upper="${ limits.upper }`;

                }
                result += '/>';

            }

            result += `${ indent1 }</joint>`

            jointNodes.push( result );

        }

    }

}
