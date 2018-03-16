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
    //   file
    // }

    static parse(object, robotname, jointfunc, meshfunc, packageprefix = 'package://') {

        const linksMap = new WeakMap();
        const meshesMap = new WeakMap();
        const texMap = new WeakMap();
        const meshes = [];
        const textures = [];
        
        let urdf = `<robot name="${robotname}">`;

        object.traverse( child => {
            
            const name = child.name;
            linksMap.set(child, name);

            let joint = '';
            let link = `<link name="${name}">`;

            if (child instanceof THREE.Mesh && child.geometry) {
                
                let meshInfo = meshesMap.get(child.geometry);
                if (!mesh) {
                    meshInfo = meshFunc(child);
                    meshesMap.set(child.geometry, meshInfo);
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

                const jointInfo = jointfunc(child);
                const { axis, type, name, limits } = jointInfo;

                joint = `<joint name="${name}" type="${type || 'fixed'}">`;
                {
                    // TODO: add position
                    // TODO: get local pos and rot relative to the parent
                    // and transformed into the URDF frame

                    joint += `<origin xyz="" rpy="" />`;

                    joint += `<parent link="${linksMap.get(child.parent)}" />`;

                    joint += `<child link="${name}" />`;

                    // TODO: Transform the axis into URDF space
                    if (axis) {
                        joint += `<axis xyz="${axis.x} ${axis.y} ${axis.z}" />`
                    }


                    // TODO: add limit
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