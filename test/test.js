/* global
    THREE URDFLoader URDFExporter
    describe it beforeAll afterAll beforeEach afterEach expect
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

    await page.coverage.startJSCoverage();
    await page.goto(`file:${ path.join(__dirname, './test-setup.html') }`);

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
                                    includesMaterials: true,

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
                            new THREE.MeshBasicMaterial()
                        );

                        const data = exp.parse(obj, () => {}, { meshFormat: 'stl' });
                        return {
                            ext: data.meshes[0].ext,
                            isDataView: data.meshes[0].data instanceof DataView,
                        };

                    });

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

        it.skip('should export the correct amount of links and joints', () => {});

        it.skip('should export joints with the correct names and attributes', () => {});

        it.skip('should be loadable by the URDFLoader', () => {});

    });

});

afterAll(async() => {

    const coverage = await page.coverage.stopJSCoverage();
    const urdfExporterCoverage = coverage.filter(o => /URDFExporter\.js$/.test(o.url));
    pti.write(urdfExporterCoverage);

    browser.close();

});
