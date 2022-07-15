const path = require('path');
const inputPath = path.join(__dirname, `./src/index.js`);
const outputPath = path.join(__dirname, `./umd/index.js`);

export default {

    input: inputPath,
    treeshake: false,
    external: p => /^three/.test(p),

    output: {

        name: 'URDFExporter',
        extend: true,
        format: 'umd',
        file: outputPath,
        sourcemap: true,

        globals: path => /^three/.test(path) ? 'THREE' : null,

    },

};
