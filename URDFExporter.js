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

    // TODO: consider a function afford skipping certain links
    // If we import then export a URDF, for example, it will add extra
    // objects for joints that shouldn't necessarily be in the description. Or
    // maybe auto collapse fixed links.

    // TODO: consider an options object instead of many function parameters

    static get STLExporter() {
        return this._stlExporter = this._stlExporter || new THREE.STLExporter();
    }

    static _defaultMeshCallback(o, linkName) {
        return {
            name: linkName,
            ext: 'stl',
            data: URDFExporter.STLExporter.parse(o)
        }
    }

    static _imageToData(image) {
        this._canvas = this._canvas || document.createElement('canvas');
        this._context = this._context || this._canvas.getContext('2D');

        this._canvas.width = image.naturalWidth;
        this._canvas.height = image.naturalHeight;

        this._context.drawImage(image, 0, 0);

        return canvas.toDataURL('image/png').replace(/^data:image\/(png|jpg);base64,/, '');
    }

    static _format(urdf) {
        const IS_END_TAG = /^<\//;
        const IS_SELF_CLOSING = /\/>$/;
        const pad = (ch, num) => (num > 0 ? ch + pad(ch, num - 1) : '');
        
        let tagnum = 0;
        return urdf
            .match(/<[^>]+>/g)
            .map(tag => {
                const res = `${pad('  ', tagnum)}${tag}`;

                if (!IS_SELF_CLOSING.test(tag)) {
                    tagnum += IS_END_TAG.test(tag) ? -1 : 1;
                }

                return res;
            })
            .join('\n');        
    }

    static parse(object, robotname, jointfunc, meshfunc = this._defaultMeshCallback, packageprefix = 'package://') {

        const linksMap = new WeakMap();
        const meshesMap = new WeakMap();
        const texMap = new WeakMap();
        const meshes = [];
        const textures = [];
        let namelessLinkCount = 0;
        let namelessJointCount = 0;

        let urdf = `<robot name="${robotname}">`;

        object.traverse( child => {
            
            const linkName = child.name || `_link_${namelessLinkCount++}`;
            linksMap.set(child, linkName);

            let joint = '';
            let link = `<link name="${linkName}">`;

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
                        link += `<mesh filename="${packageprefix}meshes/${meshInfo.name}.${meshInfo.ext}" />`
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

                            link += `<texture filename="${packageprefix}textures/${texInfo.name}.${texInfo.ext}" />`;

                        }
                    }
                    link += '</material>';
                }
                link += '</visual>'

                // TODO: add matching collision

            }
            
            link += '</link>';
           
            if (child !== object) {
                const parentName = linksMap.get(child.parent);
                const jointInfo = jointfunc(child, linkName, parentName) || {};
                const { axis, type, name, limits, effort } = jointInfo;

                joint = `<joint name="${name || `_joint_${namelessJointCount++}`}" type="${type || 'fixed'}">`;
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

        return { urdf: this._format(urdf), meshes, textures }

    }

}