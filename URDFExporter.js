/* globals THREE */
// THREE.js URDF Exporter
// http://wiki.ros.org/urdf/XML/

class URDFExporter {

    // joint func returns
    // {
    //   name
    //   type
    //   limit: { lower, upper, velocity, effort }
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

    get ColladaExporter() {

        return this._colladaExporter = this._colladaExporter || new THREE.ColladaExporter();

    }

    // Makes the provided name unique.
    // 'map' is an object with keys of already taken names
    _makeNameUnique(name, map, appendNum = 0) {

        const newName = `${ name }${ appendNum || '' }`;
        return newName in map ? this._makeNameUnique(name, map, appendNum + 1) : newName;

    }

    // Fix duplicate slashes in a file path
    _normalizePackagePath(path) {

        return path
            .replace(/[\\/]+/g, '/')
            .replace(/^package:\/*/i, 'package://');

    }

    // The default callback for generating mesh data from a link
    _defaultMeshCallback(o, linkName, preferredFormat) {

        if (preferredFormat === 'stl') {

            return {
                name: linkName,
                ext: 'stl',
                data: this.STLExporter.parse(o, { binary: true }),
                textures: [],
            };

        } else {

            // TODO: dedupe the textures here
            const res = this.ColladaExporter.parse(o, { textureDirectory: 'textures' });
            res.textures.forEach((tex, i) => {

                const newname = `${ linkName }-${ i }`;
                const nameregex = new RegExp(`${ tex.name }\\.${ tex.ext }`, 'g');

                res.data = res.data.replace(nameregex, `${ newname }.${ tex.ext }`);
                tex.name = newname;

            });

            return {
                name: linkName,
                ext: 'dae',
                data: res.data,
                textures: res.textures,
                includesMaterials: true,
            };

        }

    }

    _base64ToBuffer(str) {

        const b = atob(str);
        const buf = new Uint8Array(b.length);

        for (var i = 0, l = buf.length; i < l; i++) {

            buf[i] = b.charCodeAt(i);

        }

        return buf;

    }

    // Convert a texture to png image data
    _imageToData(image, ext) {

        this._canvas = this._canvas || document.createElement('canvas');
        this._ctx = this._ctx || this._canvas.getContext('2d');

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

        var pad = (ch, num) => (num > 0 ? ch + pad(ch, num - 1) : '');

        var tagnum = 0;
        return urdf
            .match(/(<[^>]+>[^<]+<\/[^<]+>)|(<[^>]+>)/g)
            .map(tag => {

                if (!HAS_TEXT.test(tag) && !IS_SELF_CLOSING.test(tag) && IS_END_TAG.test(tag)) {

                    tagnum--;

                }

                var res = `${ pad('  ', tagnum) }${ tag }`;
                if (!HAS_TEXT.test(tag) && !IS_SELF_CLOSING.test(tag) && !IS_END_TAG.test(tag)) {

                    tagnum++;

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
            if (type === 'fixed' && (!origin || (xyz === '0 0 0' && rpy === '0 0 0'))) {

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
                    ).length;

                // collapse the node if
                // 1. The link we'll be removing has no children so there will be no effect
                // 2. The link has children (like a visual node) making it meaningful AND there are
                // no other joints that reference this parent link, so we can move the meaningful
                // information into there

                // TODO: Consider just removing the parent node instead of the child node if the child
                // node has children. We should just remove the least complicated link.
                if (linksMap[parentName].children.length === 0 || (linksMap[childName].children.length !== 0 && parentsChildren === 1)) {

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
    parse(object, jointfunc, options = {}) {

        options = Object.assign({

            meshfunc: this._defaultMeshCallback.bind(this),
            pathPrefix: './',
            collapse: false,
            meshFormat: 'dae',
            robotName: object.name,

        }, options);

        if (options.collapse) console.warn('The "collapse" functionality isn\'t stable and my corrupt the structure of the URDF');

        const linksMap = new WeakMap(); // object > name
        const texMap = new WeakMap(); // texture > image data
        const meshes = []; // array of meshes info to save
        const textures = []; // array of texture info to save

        // used link and joint names
        const linksNameMap = {};
        const jointsNameMap = {};

        // file contents
        let urdf = `<robot name="${ options.robotName }">`;

        // use a custom travers function instead of Object3D.traverse so we
        // can stop the traversal early if we have to.
        function traverse(child) {

            const linkName = this._makeNameUnique(child.name || `_link_`, linksNameMap);
            linksNameMap[linkName] = true;
            linksMap.set(child, linkName);

            let joint = '';
            let link = `<link name="${ linkName }">`;

            // Create the link tag
            if (child instanceof THREE.Mesh) {

                // TODO: Some deduping should be happening here
                // Issue #9
                const meshInfo = options.meshfunc(child, linkName, options.meshFormat);

                if (meshInfo != null) {

                    // put the meshes in the `mesh` directory and the
                    // textures in the same directory.
                    meshInfo.directory = 'meshes/';
                    meshInfo.textures.forEach(t => {

                        t.directory =
                            `${ meshInfo.directory }${ t.directory || '' }`
                                .replace(/\\/g, '/')
                                .replace(/\/+/g, '/');

                    });

                    meshes.push(meshInfo);
                    if (meshInfo.textures) {

                        textures.push(...meshInfo.textures);

                    }

                    // Create the visual node based on the meshInfo
                    link += '<visual>';
                    {

                        link += '<origin xyz="0 0 0" rpy="0 0 0" />';

                        link += '<geometry>';
                        {

                            const meshpath = this._normalizePackagePath(`${ options.packageprefix }${ meshInfo.directory }${ meshInfo.name }.${ meshInfo.ext }`);
                            link += `<mesh filename="${ meshpath }" scale="${ child.scale.toArray().join(' ') }" />`;

                        }
                        link += '</geometry>';

                        if (!meshInfo.includesMaterials && child.material && !Array.isArray(child.material)) {

                            link += '<material name="">';
                            {

                                const col = child.material.color;
                                const rgba = `${ col.r } ${ col.g } ${ col.b } 1`;

                                link += `<color rgba="${ rgba }" />`;

                                if (child.map) {

                                    let texInfo = texMap.get(child.material.map);
                                    if (!texInfo) {

                                        const ext = 'png';
                                        texInfo = {
                                            directory: 'textures/',
                                            name: meshInfo.name,
                                            ext,
                                            data: this._imageToData(child.material.map.image, ext),
                                            original: child.material.map,
                                        };
                                        texMap.set(child.material.map, texInfo);
                                        textures.push(texInfo);

                                    }

                                    const texpath = this._normalizePackagePath(`${ options.packageprefix }textures/${ texInfo.name }.${ texInfo.ext }`);
                                    link += `<texture filename="${ texpath }" />`;

                                }

                            }
                            link += '</material>';

                        }

                    }
                    link += '</visual>';

                }

                // TODO: add matching collision

            }

            link += '</link>';

            // Create the joint tag
            if (child !== object) {

                const parentName = linksMap.get(child.parent);
                const jointInfo = jointfunc(child, linkName, parentName) || {};
                const { axis, type, name, limit } = jointInfo;

                const jointName = this._makeNameUnique(name || '_joint_', jointsNameMap);
                jointsNameMap[jointName] = true;

                joint = `<joint name="${ jointName }" type="${ type || 'fixed' }">`;
                {

                    const pos = child.position.toArray().join(' ');

                    // URDF uses fixed-axis rotations, while THREE uses moving-axis rotations
                    const euler = child.rotation.clone();
                    euler.reorder('ZYX');

                    // The last field of the array is the rotation order 'ZYX', so
                    // remove that before saving
                    const array = euler.toArray();
                    array.pop();

                    const rot = euler.toArray().join(' ');

                    joint += `<origin xyz="${ pos }" rpy="${ rot }" />`;

                    joint += `<parent link="${ parentName }" />`;

                    joint += `<child link="${ linkName }" />`;

                    if (axis) {

                        joint += `<axis xyz="${ axis.x } ${ axis.y } ${ axis.z }" />`;

                    }

                    if (limit) {

                        let limitNode = `<limit velocity="${ limit.velocity || 0 }" effort="${ limit.effort || 0 }"`;
                        if (limit.lower != null) {

                            limitNode += ` lower="${ limit.lower }"`;

                        }

                        if (limit.upper != null) {

                            limitNode += ` upper="${ limit.upper }"`;

                        }

                        limitNode += ' ></limit>';
                        joint += limitNode;

                    }

                }
                joint += '</joint>';

            }

            urdf += link;
            urdf += joint;

            // traverse all the children
            child.children.forEach(c => traverse(c));

        };

        // traverse the object
        traverse(object);

        urdf += '</robot>';

        // format the final output
        const finalurdf = this._format(options.collapse ? this._collapseLinks(urdf) : urdf);

        return { urdf: finalurdf, meshes, textures };

    }

}
