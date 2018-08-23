/* global URDFLoader URDFExporter URDFViewer THREE JSZip */
// https://stackoverflow.com/questions/19327749/javascript-blob-filename-without-link
var saveData = (function() {

    var a = document.createElement('a');
    document.body.appendChild(a);
    a.style = 'display: none';
    return function(data, fileName) {

        var blob = new Blob([ data ], { type: 'octet/stream' }),
            url = window.URL.createObjectURL(blob);
        a.href = url;
        a.download = fileName;
        a.click();
        window.URL.revokeObjectURL(url);

    };

}());

customElements.define('urdf-viewer', URDFViewer);
const exporter = new URDFExporter();
const loader = new URDFLoader();
const el = document.querySelector('urdf-viewer');

loader.load('./urdf/r2_description/robots/r2b.URDF', './urdf', robot => {

    // time out so the meshes have had time to load
    // we should have an event on the parser for when all
    // meshes have loaded

    // because we're loading from a URDFParser data here, there will be a lot of
    // extra / unneeded nodes.
    setTimeout(() => {

        // Function callback for defining the attributes for
        // the joint to be generated for the associated link.
        // If not provided, `fixed` joint type is used.
        function jointFunc(obj, childName, parentName) {

            if (obj.isURDFJoint) {

                return {
                    name: obj.name,
                    type: obj.jointType,
                    axis: obj.axis || new THREE.Vector3(0, 0, 1),
                    limit: obj.limit ? {
                        lower: obj.limit.lower,
                        upper: obj.limit.upper,
                    } : null,
                };

            }
            return {};

        }

        // Optional callback for generating a mesh for
        // a given link. If not provided, then an STL
        // exporter is used.
        function createMeshCb(obj, linkName) {

            return {
                name: linkName,
                ext: 'stl',
                data: new THREE.STLExporter().parse(obj),
                textures: [],
            };

        }

        const data = exporter.parse(robot, jointFunc, { collapse: true, createMeshCb });
        el.loadingManager.setURLModifier(url => {

            if (/urdf$/i.test(url)) return URL.createObjectURL(new Blob([data.data]));

            const mesh = data.meshes
                .filter(m => new RegExp(`${ m.directory }${ m.name }\\.${ m.ext }$`).test(url))
                .pop();

            if (mesh != null) return URL.createObjectURL(new Blob([mesh.data]));

            const tex = data.textures
                .filter(t => new RegExp(`${ t.directory }${ t.name }\\.${ t.ext }$`).test(url))
                .pop();

            if (tex != null) return URL.createObjectURL(new Blob([tex.data]));

            return url;

        });

        el.urdf = 'exported.urdf';

        const zip = new JSZip();
        zip.file('T12.URDF', data.urdf);
        data.meshes.forEach(m => zip.file(`${ m.directory }${ m.name }.${ m.ext }`, m.data));
        data.textures.forEach(t => zip.file(`${ t.directory }${ t.name }.${ t.ext }`, t.data));

        zip
            .generateAsync({ type: 'uint8array' })
            .then(zipdata => saveData(zipdata, 't12urdf.zip'));

    }, 3000);

});
