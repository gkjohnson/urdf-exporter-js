/* global
    THREE URDFLoader URDFExporter
    describe it beforeAll afterAll expect
*/
const puppeteer = require('puppeteer');
const pti = require('puppeteer-to-istanbul');
const path = require('path');

let browser = null, page = null;
beforeAll(async() => {

    browser = await puppeteer.launch({
        headless: true,

        // --no-sandbox is required to run puppeteer in Travis.
        // https://github.com/GoogleChrome/puppeteer/blob/master/docs/troubleshooting.md#running-puppeteer-on-travis-ci
        args: ['--no-sandbox'],
    });
    page = await browser.newPage();
    await page.addScriptTag({ path: path.join(__dirname, '../node_modules/three/build/three.min.js') });
    await page.addScriptTag({ path: path.join(__dirname, '../node_modules/three/examples/js/exporters/STLExporter.js') });
    await page.addScriptTag({ path: path.join(__dirname, '../node_modules/three/examples/js/exporters/ColladaExporter.js') });
    await page.addScriptTag({ path: path.join(__dirname, '../node_modules/three/examples/js/loaders/STLLoader.js') });
    await page.addScriptTag({ path: path.join(__dirname, '../node_modules/three/examples/js/loaders/ColladaLoader.js') });
    await page.addScriptTag({ path: path.join(__dirname, '../node_modules/urdf-loader/URDFLoader.js') });
    await page.addScriptTag({ path: path.join(__dirname, '../umd/URDFExporter.js') });

    await page.coverage.startJSCoverage();

    page.on('error', e => { throw new Error(e); });
    page.on('pageerror', e => { throw new Error(e); });
    page.on('console', e => {

        if (e.type() === 'error') {

            throw new Error(e.text());

        }

    });

});

describe('URDFExporter', () => {

    describe('Options', () => {

        describe('robotName', () => {

            it('should default to the name of the object', async() => {

                const res =
                    await page.evaluate(() => {

                        const exp = new URDFExporter();
                        const obj = new THREE.Object3D();
                        obj.name = 'default name';

                        return exp.parse(obj, () => {});

                    });

                expect(/<robot\s*?name="default name"/.test(res.data)).toBeTruthy();

            });

            it('should set the name of the exported robot', async() => {

                const res =
                    await page.evaluate(() => {

                        const exp = new URDFExporter();
                        const obj = new THREE.Object3D();

                        return exp.parse(obj, () => {}, { robotName: 'test' });

                    });

                expect(/<robot\s*?name="test"/.test(res.data)).toBeTruthy();

            });

        });

        describe('createMeshCb', () => {

            it('should be able to export geometry', async() => {
                const res =
                    await page.evaluate(() => {

                        const exp = new URDFExporter();
                        const obj = new THREE.Mesh(
                            new THREE.SphereBufferGeometry(),
                            new THREE.MeshBasicMaterial()
                        );

                        return exp.parse(obj, () => {}, {

                            createMeshCb: () => {

                                return {

                                    name: 'testname',
                                    ext: 'test',
                                    data: 'data',
                                    textures: [{
                                        data: 'texdata',
                                        directory: 'test/directory',
                                        name: 'texname',
                                        ext: 'png',
                                    }],

                                };

                            },

                        });

                    });

                expect(res.meshes.length).toEqual(1);
                expect(res.meshes[0].name).toEqual('testname');
                expect(res.meshes[0].ext).toEqual('test');
                expect(res.meshes[0].data).toEqual('data');
                expect(res.meshes[0].directory).toEqual('meshes/');

                expect(res.textures.length).toEqual(1);
                expect(res.textures[0].data).toEqual('texdata');
                expect(res.textures[0].name).toEqual('texname');
                expect(res.textures[0].ext).toEqual('png');
                expect(res.textures[0].directory).toEqual('meshes/test/directory');

            });

            it('should be able to export color without opacity', async() => {

                const res =
                    await page.evaluate(() => {

                        const exp = new URDFExporter();
                        const obj = new THREE.Mesh(
                            new THREE.SphereBufferGeometry(),
                            new THREE.MeshBasicMaterial()
                        );

                        return exp.parse(obj, () => {}, {

                            createMeshCb: () => {

                                return {

                                    name: 'testname',
                                    ext: 'test',
                                    data: 'data',
                                    textures: [],
                                    material: { color: new THREE.Color(1, 0, 0) },

                                };

                            },

                        });

                    });

                expect(/<color rgba="1 0 0 1"/.test(res.data)).toEqual(true);

            });

            it('should be able to export opacity without color', async() => {

                const res =
                    await page.evaluate(() => {

                        const exp = new URDFExporter();
                        const obj = new THREE.Mesh(
                            new THREE.SphereBufferGeometry(),
                            new THREE.MeshBasicMaterial()
                        );

                        return exp.parse(obj, () => {}, {

                            createMeshCb: () => {

                                return {

                                    name: 'testname',
                                    ext: 'test',
                                    data: 'data',
                                    textures: [],
                                    material: { opacity: 0.5 },

                                };

                            },

                        });

                    });

                expect(/<color rgba="1 1 1 0.5"/.test(res.data)).toEqual(true);

            });

            it('should be able to export a material with 0 opacity', async() => {

                const res =
                    await page.evaluate(() => {

                        const exp = new URDFExporter();
                        const obj = new THREE.Mesh(
                            new THREE.SphereBufferGeometry(),
                            new THREE.MeshBasicMaterial()
                        );

                        return exp.parse(obj, () => {}, {

                            createMeshCb: () => {

                                return {

                                    name: 'testname',
                                    ext: 'test',
                                    data: 'data',
                                    textures: [],
                                    material: { color: new THREE.Color(0, 0, 1), opacity: 0 },

                                };

                            },

                        });

                    });

                expect(/<color rgba="0 0 1 0"/.test(res.data)).toEqual(true);

            });

        });

        describe('pathPrefix', () => {

            it('should modify the mesh and texture path prefixes', async() => {

                const res =
                    await page.evaluate(() => {

                        const exp = new URDFExporter();
                        const obj = new THREE.Mesh(
                            new THREE.SphereBufferGeometry(),
                            new THREE.MeshBasicMaterial()
                        );

                        return exp.parse(obj, () => {}, { pathPrefix: 'package://test/' });

                    });

                expect(new RegExp('filename="package://test/meshes/_link_.dae"').test(res.data)).toBeTruthy();

            });

        });

        describe('meshFormat', () => {

            it('should export Collada', async() => {

                const res =
                    await page.evaluate(() => {

                        const exp = new URDFExporter();
                        const obj = new THREE.Mesh(
                            new THREE.SphereBufferGeometry(),
                            new THREE.MeshBasicMaterial()
                        );

                        return exp.parse(obj, () => {}, { meshFormat: 'dae' });

                    });

                expect(res.meshes[0].ext).toEqual('dae');
                expect(typeof res.meshes[0].data).toEqual('string');

            });

            it('should export STL', async() => {

                const res =
                    await page.evaluate(() => {

                        const exp = new URDFExporter();
                        const obj = new THREE.Mesh(
                            new THREE.SphereBufferGeometry(),
                            new THREE.MeshBasicMaterial({ color: 0xff0000, transparent: true, opacity: 0.5 })
                        );

                        const data = exp.parse(obj, () => {}, { meshFormat: 'stl' });
                        return {
                            ext: data.meshes[0].ext,
                            isDataView: data.meshes[0].data instanceof DataView,
                            data: data.data,
                        };

                    });

                expect(/<color rgba="1 0 0 0.5"/.test(res.data)).toEqual(true);
                expect(res.ext).toEqual('stl');
                expect(res.isDataView).toBeTruthy();

            });

            it('should default to Collada if there\'s no supported loader', async() => {

                const res =
                    await page.evaluate(() => {

                        const exp = new URDFExporter();
                        const obj = new THREE.Mesh(
                            new THREE.SphereBufferGeometry(),
                            new THREE.MeshBasicMaterial()
                        );

                        return exp.parse(obj, () => {}, { meshFormat: 'gltf' });

                    });

                expect(res.meshes[0].ext).toEqual('dae');
                expect(typeof res.meshes[0].data).toEqual('string');

            });

        });

        describe('collapse', () => {

            it('should retain unnecessary fixed joints if `false`', async() => {

                const res =
                    await page.evaluate(() => {

                        const exp = new URDFExporter();

                        const obj1 = new THREE.Object3D();
                        const obj2 = new THREE.Object3D();
                        const obj3 = new THREE.Object3D();
                        const obj4 = new THREE.Object3D();
                        const mesh = new THREE.Mesh(
                            new THREE.SphereBufferGeometry(),
                            new THREE.MeshBasicMaterial()
                        );

                        obj1.add(obj2);
                        obj2.add(obj3);
                        obj3.add(obj4);
                        obj4.add(mesh);

                        return exp.parse(obj1, () => {}, { collapse: false });

                    });

                expect(res.data.match(/<link/g).length).toEqual(5); // 5 links

            });

            it('should remove unnecessary fixed joints if `true`', async() => {

                const res =
                    await page.evaluate(() => {

                        const exp = new URDFExporter();

                        const obj1 = new THREE.Object3D();
                        const obj2 = new THREE.Object3D();
                        const obj3 = new THREE.Object3D();
                        const obj4 = new THREE.Object3D();
                        const mesh = new THREE.Mesh(
                            new THREE.SphereBufferGeometry(),
                            new THREE.MeshBasicMaterial()
                        );

                        obj1.add(obj2);
                        obj2.add(obj3);
                        obj3.add(obj4);
                        obj4.add(mesh);

                        return exp.parse(obj1, () => {}, { collapse: true });

                    });

                expect(res.data.match(/<link/g).length).toEqual(1); // 1 links

            });

            it('should keep fixed joints with transform data if `true`', async() => {

                const res =
                    await page.evaluate(() => {

                        const exp = new URDFExporter();

                        const obj1 = new THREE.Object3D();
                        const obj2 = new THREE.Object3D();
                        const obj3 = new THREE.Object3D();
                        const obj4 = new THREE.Object3D();
                        const mesh = new THREE.Mesh(
                            new THREE.SphereBufferGeometry(),
                            new THREE.MeshBasicMaterial()
                        );

                        obj1.add(obj2);
                        obj2.add(obj3);
                        obj3.position.y = 1;
                        obj3.add(obj4);
                        obj4.add(mesh);

                        return exp.parse(obj1, () => {}, { collapse: true });

                    });

                expect(res.data.match(/<link/g).length).toEqual(2); // 2 links

            });

        });

    });

    describe('parse', () => {

        beforeAll(async() => {

            await page.evaluate(() => {

                window.parseURDF = function(exportedURDF) {

                    return new Promise(resolve => {

                        function simplifyHierarchy(obj) {

                            const simple = {};
                            simple.type = obj.type;
                            simple.name = obj.name;
                            simple.axis = obj.axis ? obj.axis.toArray() : null;
                            simple.jointType = obj.jointType || null;
                            simple.limit = obj.limit || null;
                            simple.children = obj.children.map(c => simplifyHierarchy(c));

                            return simple;

                        }

                        const loader = new URDFLoader();
                        const colladaLoader = new THREE.ColladaLoader();
                        loader.parse(exportedURDF.data, '', r => {

                            requestAnimationFrame(() => resolve(simplifyHierarchy(r)));

                        }, {
                            loadMeshCb: (path, ext, done) => {

                                const data = exportedURDF.meshes.filter(d => path.indexOf(`${ d.directory }${ d.name }.${ d.ext }`) !== -1)[0];
                                done(colladaLoader.parse(data.data).scene.children[0]);

                            },
                        });

                    });

                };

            });

        });

        it('should export the correct amount of links and joints', async() => {

            const res =
                await page.evaluate(async() => {

                    const exp = new URDFExporter();

                    const obj1 = new THREE.Object3D();
                    const obj2 = new THREE.Object3D();
                    const mesh = new THREE.Mesh(
                        new THREE.SphereBufferGeometry(),
                        new THREE.MeshBasicMaterial()
                    );

                    obj1.add(obj2);
                    obj2.add(mesh);

                    const exported = exp.parse(obj1, () => {});
                    return window.parseURDF(exported);

                });

            const robot = res;
            expect(robot.type).toEqual('URDFRobot');
            expect(robot.children.length).toEqual(1);
            expect(robot.axis).toEqual(null);
            expect(robot.jointType).toEqual(null);
            expect(robot.limit).toEqual(null);
            expect(robot.children.length).toEqual(1);

            const link1 = robot.children[0];
            expect(link1.type).toEqual('URDFLink');
            expect(link1.name).toEqual('_link_');
            expect(link1.children.length).toEqual(1);

            const joint1 = link1.children[0];
            expect(joint1.type).toEqual('URDFJoint');
            expect(joint1.children.length).toEqual(1);
            expect(joint1.axis).toEqual(null);
            expect(joint1.jointType).toEqual('fixed');
            expect(joint1.limit).toEqual({ lower: 0, upper: 0 });
            expect(joint1.children.length).toEqual(1);

            const link2 = joint1.children[0];
            expect(link2.type).toEqual('URDFLink');
            expect(link2.name).toEqual('_link_1');
            expect(link2.children.length).toEqual(1);

            const joint2 = link2.children[0];
            expect(joint2.type).toEqual('URDFJoint');
            expect(joint2.children.length).toEqual(1);
            expect(joint2.axis).toEqual(null);
            expect(joint2.jointType).toEqual('fixed');
            expect(joint2.limit).toEqual({ lower: 0, upper: 0 });
            expect(joint2.children.length).toEqual(1);

            const link3 = joint2.children[0];
            expect(link3.type).toEqual('URDFLink');
            expect(link3.name).toEqual('_link_2');
            expect(link3.children.length).toEqual(1);

            const mesh = link3.children[0];
            expect(mesh.type).toEqual('Mesh');
            expect(mesh.children.length).toEqual(0);

        });

        it('should export joints with the correct names and attributes', async() => {

            const res =
                await page.evaluate(async() => {

                    const exp = new URDFExporter();

                    const obj = new THREE.Object3D();
                    const mesh = new THREE.Mesh(
                        new THREE.SphereBufferGeometry(),
                        new THREE.MeshBasicMaterial()
                    );

                    obj.name = 'object';
                    mesh.name = 'mesh';

                    obj.add(mesh);

                    const exported = exp.parse(obj, (o, childName, parentName) => ({

                        name: `${ parentName }2${ childName }`,
                        limit: { lower: -100, upper: 100 },
                        type: 'revolute',
                        axis: new THREE.Vector3(0, 0, 1),

                    }));

                    return window.parseURDF(exported);

                });

            const link1 = res.children[0];
            expect(link1.name).toEqual('object');
            expect(link1.type).toEqual('URDFLink');
            expect(link1.children.length).toEqual(1);

            const joint1 = link1.children[0];
            expect(joint1.name).toEqual('object2mesh');
            expect(joint1.type).toEqual('URDFJoint');
            expect(joint1.children.length).toEqual(1);
            expect(joint1.axis).toEqual([0, 0, 1]);
            expect(joint1.jointType).toEqual('revolute');
            expect(joint1.limit).toEqual({ lower: -100, upper: 100 });
            expect(joint1.children.length).toEqual(1);

            const link2 = joint1.children[0];
            expect(link2.type).toEqual('URDFLink');
            expect(link2.name).toEqual('mesh');
            expect(link2.children.length).toEqual(1);

            const mesh = link2.children[0];
            expect(mesh.type).toEqual('Mesh');
            expect(mesh.children.length).toEqual(0);

        });

        it('should stop traversal early if `isLeaf` is returned as `true`', async() => {

            const res =
                await page.evaluate(async() => {

                    const exp = new URDFExporter();

                    const obj1 = new THREE.Object3D();
                    const obj2 = new THREE.Object3D();
                    const obj3 = new THREE.Object3D();
                    const obj4 = new THREE.Object3D();
                    const obj5 = new THREE.Object3D();
                    const obj6 = new THREE.Object3D();
                    const mesh = new THREE.Mesh(
                        new THREE.SphereBufferGeometry(),
                        new THREE.MeshBasicMaterial()
                    );

                    obj1.add(obj2);
                    obj2.add(obj3);
                    obj3.add(obj4);
                    obj4.add(obj5);
                    obj5.add(obj6);
                    obj6.add(mesh);

                    const exported = exp.parse(obj1, () => ({ isLeaf: true }));
                    return window.parseURDF(exported);

                });

            const link1 = res.children[0];
            expect(link1.type).toEqual('URDFLink');

            const joint1 = link1.children[0];
            expect(joint1.type).toEqual('URDFJoint');

            const link2 = joint1.children[0];
            expect(link2.type).toEqual('URDFLink');

            const colladaRoot = link2.children[0];
            expect(colladaRoot.type).toEqual('Group');
            expect(colladaRoot.children.length).toEqual(1);

        });

        it.skip('should modify the directory of textures included with the exported meshes', () => {});

        it.skip('should include material properties and textures when `material` is included from the mesh generation function', () => {});

        afterAll(async() => {

            await page.evaluate(() => delete window.parseURDF);

        });

    });

});

afterAll(async() => {

    const coverage = await page.coverage.stopJSCoverage();
    const urdfExporterCoverage = coverage.filter(o => /URDFExporter\.js$/.test(o.url));
    pti.write(urdfExporterCoverage);

    browser.close();

});
