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

    static get STLExporter() {
        return this._stlExporter = this._stlExporter || new THREE.STLExporter();
    }

    // Makes the provided name unique.
    // 'map' is an object with keys of already taken names
    static _makeNameUnique(name, map, appendNum = 0) {
        const newName = `${ name }${ appendNum ? appendNum : '' }`;
        return newName in map ? this._makeNameUnique(name, map, appendNum + 1) : newName;
    }

    // Fix duplicate slashes in a file path
    static _normalizePackagePath(path) {
        return path
            .replace(/[\\/]+/g, '/')
            .replace(/^package:\/*/i, 'package://');
    }

    // The default callback for generating mesh data from a link
    static _defaultMeshCallback(o, linkName) {
        return {
            name: linkName,
            ext: 'stl',
            data: URDFExporter.STLExporter.parse(o)
        }
    }

    // Convert a texture to png image data
    static _imageToData(image) {
        this._canvas = this._canvas || document.createElement('canvas');
        this._context = this._context || this._canvas.getContext('2D');

        this._canvas.width = image.naturalWidth;
        this._canvas.height = image.naturalHeight;

        this._context.drawImage(image, 0, 0);

        return canvas.toDataURL('image/png').replace(/^data:image\/(png|jpg);base64,/, '');
    }

    // Convert the urdf xml into a well-formatted, indented format
    static _format(urdf) {
        const IS_END_TAG = /^<\//;
        const IS_SELF_CLOSING = /\/>$/;
        const pad = (ch, num) => (num > 0 ? ch + pad(ch, num - 1) : '');
        
        let tagnum = 0;
        return urdf
            .match(/<[^>]+>/g)
            .map(tag => {
                if (!IS_SELF_CLOSING.test(tag) && IS_END_TAG.test(tag)) {
                    tagnum --;
                }

                const res = `${pad('  ', tagnum)}${tag}`;

                if (!IS_SELF_CLOSING.test(tag) && !IS_END_TAG.test(tag)) {
                    tagnum ++;
                }

                return res;
            })
            .join('\n');        
    }

    // Convert the object into a urdf and get the associated
    // mesh and texture data
    static parse(object, robotname, jointfunc, meshfunc = this._defaultMeshCallback, packageprefix = 'package://') {

        const linksMap = new WeakMap();     // object > name
        const meshesMap = new WeakMap();    // geometry > mesh data
        const texMap = new WeakMap();       // texture > image data
        const meshes = [];                  // array of meshes info to save
        const textures = [];                // array of texture info to save

        // used link and joint names
        let linksNameMap = {};
        let jointsNameMap = {};

        let urdf = `<robot name="${robotname}">`;


        const stack = [object];
        let parent = null;
        while (stack.length) {
            const child = stack.pop();
            stack.push(...child.children)

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
                                texInfo = {
                                    name: meshInfo.name,
                                    ext: 'png',
                                    data: this._imageToData(child.material.map.image)
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
            if (parent) {
                const parentName = linksMap.get(parent);
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

            parent = child;
        }

        urdf += '</robot>';

        return { urdf: this._format(urdf), meshes, textures }

    }

}