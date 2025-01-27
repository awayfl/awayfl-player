import typescript from 'rollup-plugin-typescript2'
import nodeResolve from '@rollup/plugin-node-resolve';
import commonjs from '@rollup/plugin-commonjs';
import { terser } from 'rollup-plugin-terser';
import gzip from 'rollup-plugin-gzip';

export default [
	{
		input: './dist/index.js',
		output: {
			name: 'awayflplayer',
			sourcemap: true,
			format: 'iife',
			file: './bundle/awayfl-player.umd.js',
		},
		plugins: [
			nodeResolve(),
			// {
			//     transform(code, id) {
			//         return code.replace(/\/\*\* @class \*\//g, "\/*@__PURE__*\/");
			//     }
			// },
			commonjs(),
			terser({
				// mangle: {
				// 	properties: {
				// 		reserved: ['startPokiGame', 'userAgent', 'Number', '__constructor__', 'prototype']
				// 	}
				// }
			}),
			gzip()
		]
	},
	{
		input: './lib/nape/index.ts',
		output: {
			name: 'AWAY_NAPE',
			sourcemap: false,
			format: 'iife',
			file: './template/away_nape.js'
		},
		plugins: [
			typescript(),
			nodeResolve(),
			// {
			//     transform(code, id) {
			//         return code.replace(/\/\*\* @class \*\//g, "\/*@__PURE__*\/");
			//     }
			// },
			commonjs(),
			terser({
				// mangle: {
				// 	properties: {
				// 		reserved: ['startPokiGame', 'userAgent', 'Number', '__constructor__', 'prototype']
				// 	}
				// }
			}),
			gzip()
		]
	}
]