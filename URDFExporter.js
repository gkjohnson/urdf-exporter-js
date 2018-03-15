class URDFExporter {

    // joint func returns
    // {
    //   name
    //   type
    //   limits: { min, max }
    //   axis
    // }

    // mesh func returns
    // {
    //   name
    //   file
    // }

    static parse(object, robotname, jointfunc, meshfunc) {

        const linksMap = new WeakMap();
        const meshesMap = new WeakMap();
        const meshes = [];
        
        let urdf = `<robot name="${robotname}">`;

        object.traverse( child => {
            
            const name = child.name;
            linksMap.set(child.parent, name);

            let joint = '';
            let link = `<link name="${name}">`;

            if (child instanceof THREE.Mesh) {
                
                // TODO: add visual node if mesh

                // TODO: add matching collision
            
            }
            
            link += '</link>';
           
            if (child !== object) {
                joint = `<joint name="" type="">`;

                // TODO: add position

                // TODO: add parent

                // TODO: add child

                // TODO: add axis

                // TODO: add limit

                joint += '</joint>';
            }

            urdf += link;
            urdf += joint;
        });

        urdf += '</robot>';

        return { urdf, meshes }

    }

}