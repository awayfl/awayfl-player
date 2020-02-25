var includePaths = require('rollup-plugin-includepaths');
var commonjs = require('rollup-plugin-commonjs');
var nodeResolve = require('rollup-plugin-node-resolve');

module.exports = {
	input: './dist/index.js',
	output: {
		name: 'AwayflPlayer',
		sourcemap: true,
		format: 'umd',
		file: './bundle/awayfl-player.umd.js',
		globals: {
			'@awayfl/avm1': 'AwayflAvm1',
			'@awayfl/avm2': 'AwayflAvm2',
			'@awayfl/playerglobal': 'AwayflPlayerglobal',
			'@awayjs/core': 'AwayjsCore',
			'@awayjs/graphics': 'AwayjsGraphics',
			'@awayjs/materials': 'AwayjsMaterials',
			'@awayjs/renderer': 'AwayjsRenderer',
			'@awayjs/scene': 'AwayjsScene',
			'@awayjs/stage': 'AwayjsStage',
			'@awayjs/swf-viewer': 'AwayjsSwfViewer',
			'@awayjs/view': 'AwayjsView',
		},
	},
	external: [
		'@awayfl/avm1',
		'@awayfl/avm2',
		'@awayfl/playerglobal',
		'@awayjs/core',
		'@awayjs/graphics',
		'@awayjs/materials',
		'@awayjs/renderer',
		'@awayjs/scene',
		'@awayjs/stage',
		'@awayjs/swf-viewer',
		'@awayjs/view',
	],
	plugins: [
		nodeResolve({
			jsnext: true,
			main: true,
			module: true
		}) ]
};