class URDFExporter {

    // joint func returns
    // {
    //   name
    //   type
    //   limits: { lower, upper }
    //   axis
    // }

    // mesh func returns
    // {
    //   filename
    //   data
    // }

    // TODO: consider a function afford skipping certain links
    // If we import then export a URDF, for example, it will add extra
    // objects for joints that shouldn't necessarily be in the description. Or
    // maybe auto collapse fixed links.

    // TODO: consider an options object instead of many function parameters

    static parse(object, robotname, jointfunc, meshfunc, packageprefix = 'package://') {

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
            console.log(child, linkName)
            linksMap.set(child, linkName);

            let joint = '';
            let link = `<link name="${linkName}">`;

            if (child instanceof THREE.Mesh && child.geometry) {
                
                let meshInfo = meshesMap.get(child.geometry);
                if (!meshInfo) {
                    meshInfo = meshfunc(child);
                    meshesMap.set(child.geometry, meshInfo);
                    meshes.push(meshInfo);
                }

                link += '<visual>';
                {
                    link += '<origin xyz="0 0 0" rpy="0 0 0" />';
                    
                    link += '<geometry>';
                    {
                        link += `<mesh filename="${packageprefix}/meshes/${meshInfo.filename}" />`
                    }
                    link += '</geometry>';
                    
                    link += '<material name="">';
                    {
                        const col = child.material.color;
                        const rgba = `${col.r} ${col.g} ${col.b} 1`;

                        link += `<color rgba="${rgba}" />`

                        // TODO: add texture if needed and write texture
                        // to array
                    }
                    link += '</material>';
                }
                link += '</visual>'

                // TODO: add matching collision

            }
            
            link += '</link>';
           
            if (child !== object) {

                const jointInfo = jointfunc(child) || {};
                const { axis, type, name, limits } = jointInfo;
                const name = jointInfo.name ;

                joint = `<joint name="${name || `_joint_${namelessJointCount++}`}" type="${type || 'fixed'}">`;
                {
                    const pos = `${child.position.x} ${child.position.y} ${child.position.z}`;
                    
                    // URDF uses fixed-axis rotations, while THREE uses moving-axis rotations
                    const euler = child.euler.clone();
                    euler.order = 'ZYX';
                    const rot = `${euler.x} ${euler.y} ${euler.z}`

                    joint += `<origin xyz="${pos}" rpy="${rot}" />`;

                    joint += `<parent link="${linksMap.get(child.parent)}" />`;

                    joint += `<child link="${linkName}" />`;

                    // TODO: Transform the axis into URDF space
                    if (axis) {
                        joint += `<axis xyz="${axis.x} ${axis.y} ${axis.z}" />`
                    }

                    if (limits) {

                        let limit = '<limit ';
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

        return { urdf, meshes, textures }

    }

}