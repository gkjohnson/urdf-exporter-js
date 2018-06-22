// THREE.js URDF Exporter
// http://wiki.ros.org/urdf/XML/

class URDFExporter {

    // joint func returns
    // {
    //   name
    //   type
    //   limits: { lower, upper, velocity, effort }
    //   axis
    // }

    // mesh func returns
    // {
    //   name,
    //   ext,
    //   data
    // }

    get STLExporter() {
        return this._stlExporter = this._stlExporter || new THREE.STLExporter();
    }

    // Makes the provided name unique.
    // 'map' is an object with keys of already taken names
    _makeNameUnique(name, map, appendNum = 0) {
        const newName = `${ name }${ appendNum ? appendNum : '' }`;
        return newName in map ? this._makeNameUnique(name, map, appendNum + 1) : newName;
    }

    // Fix duplicate slashes in a file path
    _normalizePackagePath(path) {
        return path
            .replace(/[\\/]+/g, '/')
            .replace(/^package:\/*/i, 'package://');
    }

    // The default callback for generating mesh data from a link
    _defaultMeshCallback(o, linkName) {
        // TODO: Include a DAE exporter?
        return {
            name: linkName,
            ext: 'stl',
            data: this.STLExporter.parse(o, { binary: true })
        }
    }

    _base64ToBuffer(str) {
        const b = atob(str);
        const buf = new Uint8Array(b.length);

        for (var i = 0, l = buf.length; i < l; i ++) {
            buf[i] = b.charCodeAt(i);
        }

        return buf;
    }

    // Convert a texture to png image data
    _imageToData(image, ext) {

            this._canvas = this._canvas || document.createElement('canvas');
            this._ctx = this._ctx || canvas.getContext('2d');

            const canvas = this._canvas;
            const ctx = this._ctx;

            canvas.width = image.naturalWidth;
            canvas.height = image.naturalHeight;

            ctx.drawImage(image, 0, 0);

            // Get the base64 encoded data
            const base64data = canvas
                        .toDataURL(`image/${ ext }`, 1)
                        .replace(/^data:image\/(png|jpg);base64,/, '');

            // Convert to a uint8 array
            return this._base64ToBuffer(base64data);

        }

    // Convert the urdf xml into a well-formatted, indented format
    _format(urdf) {
        var IS_END_TAG = /^<\//;
        var IS_SELF_CLOSING = /(\?>$)|(\/>$)/;
        var HAS_TEXT = /<[^>]+>[^<]*<\/[^<]+>/;

        var pad = ( ch, num ) => ( num > 0 ? ch + pad( ch, num - 1 ) : '' );

        var tagnum = 0;
        return urdf
            .match( /(<[^>]+>[^<]+<\/[^<]+>)|(<[^>]+>)/g )
            .map( tag => {
                if ( ! HAS_TEXT.test( tag ) && ! IS_SELF_CLOSING.test( tag ) && IS_END_TAG.test( tag ) ) {
                    tagnum --;
                }

                var res = `${ pad( '  ', tagnum ) }${ tag }`;
                if ( ! HAS_TEXT.test( tag ) && ! IS_SELF_CLOSING.test( tag ) && ! IS_END_TAG.test( tag ) ) {
                    tagnum ++;

                }

                return res;
            })
            .join('\n');
    }

    // Remove any unnecessary joints and links that fixed and have identity transforms
    _collapseLinks(urdf) {
        const xmlDoc = (new DOMParser()).parseFromString(urdf, 'text/xml');
        const robottag = xmlDoc.children[0];

        // cache the children as an array
        const children = [...robottag.children];

        // get the list of links indexed by name
        const linksMap = {};
        const links = children.filter(t => t.tagName.toLowerCase() === 'link');
        links.forEach(l => linksMap[l.getAttribute('name')] = l);

        // remove the unnecessary joints
        const joints = children.filter(t => t.tagName.toLowerCase() === 'joint');

        // TODO: Do we need to traverse in reverse so as nodes are removed, they're taken into
        // account in subsequent collapses? Order is important here.
        joints.forEach(j => {
            const origin = [...j.children].filter(t => t.tagName.toLowerCase() === 'origin')[0];
            const type = j.getAttribute('type') || 'fixed';

            // if the node is fixed and has an identity transform then we can remove it
            const xyz = origin.getAttribute('xyz') || '0 0 0';
            const rpy = origin.getAttribute('rpy') || '0 0 0';
            if (type === 'fixed' && (!origin || xyz === '0 0 0' && rpy === '0 0 0')) {

                const childName =
                    [...j.children]
                        .filter(t => t.tagName.toLowerCase() === 'child')[0]
                        .getAttribute('link');

                const parentName =
                    [...j.children]
                        .filter(t => t.tagName.toLowerCase() === 'parent')[0]
                        .getAttribute('link');

                // how many child joints reference the same joint as this parent
                const parentsChildren =
                    joints.filter(j2 =>
                        [...j.children]
                            .filter(t => t.tagName.toLowerCase() === 'parent')
                            .filter(t => t.getAttribute('link') === parentName)
                            .length !== 0
                    ).length

                // collapse the node if
                // 1. The link we'll be removing has no children so there will be no effect
                // 2. The link has children (like a visual node) making it meaningful AND there are
                // no other joints that reference this parent link, so we can move the meaningful
                // information into there

                // TODO: Consider just removing the parent node instead of the child node if the child
                // node has children. We should just remove the least complicated link.
                if (linksMap[parentName].children.length === 0 || linksMap[childName].children.length !== 0 && parentsChildren === 1) {

                    if (linksMap[childName].children.length) {
                        [...linksMap[childName].children].forEach(c => linksMap[parentName].appendChild(c));
                    }

                    // find joints that have this joint as the parent and move it to the parent
                    joints.forEach(j2 => 
                        [...j2.children]
                            .filter(t => t.tagName.toLowerCase() === 'parent')
                            .filter(t => t.getAttribute('link') === childName)
                            .forEach(t => t.setAttribute('link', parentName))
                    );

                    // remove this joint from the robot
                    robottag.removeChild(j);
                }
            }
        });

        // remove any links that arent referenced by the existing joints
        [...robottag.children]
            .filter(t => t.tagName.toLowerCase() === 'joint')
            .forEach(j => {
                
                const childName =
                    [...j.children]
                        .filter(t => t.tagName.toLowerCase() === 'child')[0]
                        .getAttribute('link');

                const parentName =
                    [...j.children]
                        .filter(t => t.tagName.toLowerCase() === 'parent')[0]
                        .getAttribute('link');

                delete linksMap[childName];
                delete linksMap[parentName];

            });

        // the links remaining aren't being referenced by any
        // joints and can be removed
        for (const name in linksMap) robottag.removeChild(linksMap[name]);

        return new XMLSerializer().serializeToString(xmlDoc.documentElement);
    }

    // Convert the object into a urdf and get the associated
    // mesh and texture data
    parse(object, robotname, jointfunc, options = {}) {
        
        const meshfunc = options.createMesh || this._defaultMeshCallback;
        const packageprefix = options.packagePrefix || 'package://';
        const collapse = options.collapse || false;

        if (collapse) console.warn('The "collapse" functionality isn\'t stable and my corrupt the structure of the URDF');

        const linksMap = new WeakMap();     // object > name
        const meshesMap = new WeakMap();    // geometry > mesh data
        const texMap = new WeakMap();       // texture > image data
        const meshes = [];                  // array of meshes info to save
        const textures = [];                // array of texture info to save

        // used link and joint names
        let linksNameMap = {};
        let jointsNameMap = {};

        let urdf = `<robot name="${robotname}">`;
        object.traverse(child => {
            const linkName = this._makeNameUnique(child.name || `_link_`, linksNameMap);
            linksNameMap[linkName] = true;
            linksMap.set(child, linkName);

            let joint = '';
            let link = `<link name="${linkName}">`;

            // Create the link tag
            if (child instanceof THREE.Mesh && child.geometry) {
                
                let meshInfo = meshesMap.get(child.geometry);
                if (!meshInfo) {
                    meshInfo = meshfunc(child, linkName);
                    meshesMap.set(child.geometry, meshInfo);
                    meshes.push(meshInfo);
                }

                link += '<visual>';
                {
                    link += '<origin xyz="0 0 0" rpy="0 0 0" />';
                    
                    link += '<geometry>';
                    {
                        const meshpath = this._normalizePackagePath(`${packageprefix}/meshes/${meshInfo.name}.${meshInfo.ext}`);
                        link += `<mesh filename="${meshpath}" />`
                    }
                    link += '</geometry>';
                    
                    link += '<material name="">';
                    {
                        const col = child.material.color;
                        const rgba = `${col.r} ${col.g} ${col.b} 1`;

                        link += `<color rgba="${rgba}" />`;

                        if (child.map) {

                            let texInfo = texMap.get(child.material.map);
                            if (!texInfo) {
                                const ext = 'png';
                                texInfo = {
                                    name: meshInfo.name,
                                    ext,
                                    data: this._imageToData(child.material.map.image, ext)
                                }
                                texMap.set(child.material.map, texInfo);
                                textures.push(texInfo);
                            }

                            const texpath = this._normalizePackagePath(`${packageprefix}/textures/${texInfo.name}.${texInfo.ext}`);
                            link += `<texture filename="${texpath}" />`;

                        }
                    }
                    link += '</material>';
                }
                link += '</visual>'

                // TODO: add matching collision

            }
            
            link += '</link>';
           
            // Create the joint tag
            if (child !== object) {
                const parentName = linksMap.get(child.parent);
                const jointInfo = jointfunc(child, linkName, parentName) || {};
                const { axis, type, name, limits, effort } = jointInfo;

                const jointName = this._makeNameUnique(name || '_joint_', jointsNameMap);
                jointsNameMap[jointName] = true;

                joint = `<joint name="${jointName}" type="${type || 'fixed'}">`;
                {
                    const pos = `${child.position.x} ${child.position.y} ${child.position.z}`;
                    
                    // URDF uses fixed-axis rotations, while THREE uses moving-axis rotations
                    const euler = child.rotation.clone();
                    euler.order = 'ZYX';
                    const rot = `${euler.x} ${euler.y} ${euler.z}`

                    joint += `<origin xyz="${pos}" rpy="${rot}" />`;

                    joint += `<parent link="${parentName}" />`;

                    joint += `<child link="${linkName}" />`;

                    if (axis) {
                        joint += `<axis xyz="${axis.x} ${axis.y} ${axis.z}" />`
                    }

                    if (limits) {

                        let limit = `<limit velocity="${limits.velocity || 0}" effort="${limits.effort || 0}" `;
                        if (limits.lower != null) {
                            limit += `lower="${limits.lower}"`;
                        }

                        if (limits.upper != null) {
                            limit += `upper="${limits.upper}"`;
                        }

                        limit += ' />';
                    }
                }
                joint += '</joint>';
            }

            urdf += link;
            urdf += joint;
        });

        urdf += '</robot>';

        // format the final output
        const finalurdf = this._format(collapse ? this._collapseLinks(urdf) : urdf);
        
        return { urdf: finalurdf, meshes, textures }

    }

}
