const path = require('path');
const fs = require('fs');
const webpack = require('webpack');
const CopyWebPackPlugin = require('copy-webpack-plugin');
const HTMLWebPackPlugin = require('html-webpack-plugin');
const Terser = require('terser-webpack-plugin')
const rimraf = require("rimraf");
const tsloader = require.resolve('ts-loader');
const merge = require("webpack-merge").merge;
const config = require('./awayfl.config.js')

module.exports = (env = {}) => {

	var isProd = !!env.prod;

	// force some configs dependant on prod and dev
	config.rt_debug = isProd ? false : config.rt_debug;

	config.rt_showFPS = isProd ? false : config.rt_showFPS;

	config.cacheBuster = isProd ? false : config.cacheBuster;

	config.allowURLSearchParams = isProd ? false : config.allowURLSearchParams;

	// split mode right now errors in watch mode
	config.split = isProd ? config.split : false;

	if (config.debugConfig) {
		console.log("global config used for webpack:");
		for (var key in config) {
			console.log("	- config." + key, config[key]);
		}
	}

	const entry = {};
	entry[config.entryName] = [config.entryPath];

	let plugins = processConfig(config, __dirname, CopyWebPackPlugin, HTMLWebPackPlugin, webpack.BannerPlugin, fs, rimraf, path);

	const common = {

		entry: entry,

		output: {
			pathinfo: false,
			path: path.join(__dirname, "bin"),
			filename: 'js/[name].js'
		},
		resolve: {
			alias: {},
			// Add `.ts` and `.tsx` as a resolvable extension.
			extensions: ['.webpack.js', '.web.js', '.js', '.ts', '.tsx']
		},
		module: {
			rules: [
				{
					test: /\.ts(x?)/,
					exclude: /node_modules/,
					loader: tsloader,
					options: {
						experimentalWatchApi: true,
						transpileOnly: true
					}
				},
			]
		},
		plugins: plugins,

		performance: {
			hints: false // wp4
		},
		stats: {
			cached: true, // wp4
			errorDetails: true, // wp4
			colors: true // wp4
		},
		devServer: {
			client: {
				progress: true, // wp5
			}
		},
	}

	const dev = {
		target: 'web',
		mode: "development",// wp4
		//devtool: 'source-map',
		devtool: 'cheap-module-source-map',//use this option for recompiling libs
		devServer: {
			static: {
				publicPath: "/",
			},
			open: false,
			hot: false,
			watchFiles: ['src/**/*.*'],
			client: {
				progress: true,
			},
			allowedHosts: "all",
			port: 80,
		},
		optimization: {
			//minimize: false // wp4
		}
	}

	const prod = {
		mode: "production",// wp4
		bail: true
	};

	if(Terser) {
		prod.optimization = {
			minimize: true,
			minimizer: [
				new Terser({
				  extractComments: {
					condition: /^\**!|@preserve|@license|@cc_on/i,
					filename: 'LICENSES.txt'
				  },
				}),
			],
		}
	} else {
		console.warn("TERSER IS REQUIRE FOR REMOVING COMMENTS!");
	}

	return merge(common, isProd ? prod : dev);

}

// process config
// return a list of webpack-plugins
const processConfig = (config, rootPath, CopyWebPackPlugin, HTMLWebPackPlugin, BannerPlugin, fs, rimraf, path) => {

	var plugins = [];

	// 	if no split, copy as3 buildins to asset folder
	//	if split, we will copy them for each game-config individually
	if (config.buildinsPath && config.buildinsPath.length && !config.split) {
		plugins.push(new CopyWebPackPlugin({
			patterns: [
				{ from: config.buildinsPath, to: 'assets/builtins' },
			],
		}));
	}

	//	copy loader.js to js-folder
	//	if split, this will be copied to the subfolder together with webpack-bundel
	plugins.push(new CopyWebPackPlugin({
		patterns: [
			{ from: config.loaderTemplate, to: 'js' },
		],
	}));

	// collect all game-urls to create a index.html:
	let gameURLS = {};

	// map to collect copied resources, so we preent any redunant copies
	let copiedResources = {};

	var _loop_1 = function (i) {

		var fileConfig = config.fileconfigs[i];
		var folderName = fileConfig.rt_filename;
		var outputPath = config.split ? folderName + "/" : "";

		//	if split, copy buildins to each output folder:

		if (config.buildinsPath && config.buildinsPath.length && config.split) {
			plugins.push(new CopyWebPackPlugin({
				patterns: [
					{ from: config.buildinsPath, to: outputPath + 'assets/builtins' },
				],
			}));
		}

		// get config for this file merged with default values from global config:

		var configForHTML = getConfig(fileConfig, config);

		//	copy assets for this file-config:

		swfPath = path.join(rootPath, "src", "assets", configForHTML.filename + ".swf");
		if (!fs.existsSync(swfPath)) {
			throw new Error("invalid filename path for fileconfig " + configForHTML.filename);
		}
		stats = fs.statSync(swfPath);
		filesize = stats["size"];
		if (!fs.existsSync(path.join(rootPath, "src", "assets", configForHTML.splash))) {
			throw new Error("invalid splashscreen path for fileconfig " + configForHTML.splash);
		}

		plugins.push(new CopyWebPackPlugin({
			patterns: [
				{ from: swfPath, to: outputPath + "assets" },
			],
		}));

		plugins.push(new CopyWebPackPlugin({
			patterns: [
				{ from: path.join(rootPath, "src", "assets", configForHTML.splash), to: outputPath + "assets" },
			],
		}));

		//	optional copy loading image:

		if (configForHTML.loading) {
			if (!fs.existsSync(path.join(rootPath, "src", "assets", configForHTML.loading.image))) {
				throw ("invalid loading image path for fileconfig " + configForHTML.loading.image);
			}
			plugins.push(new CopyWebPackPlugin({
				patterns: [
					{ from: path.join(rootPath, "src", "assets", configForHTML.loading.image), to: outputPath + "assets" },
				],
			}));
		}


		//	optional copy start image:

		if (configForHTML.start) {
			if (!fs.existsSync(path.join(rootPath, "src", "assets", configForHTML.start.image))) {
				throw ("invalid start image path for fileconfig " + configForHTML.start.image);
			}
			plugins.push(new CopyWebPackPlugin({
				patterns: [
					{ from: path.join(rootPath, "src", "assets", configForHTML.start.image), to: outputPath + "assets" },
				],
			}));
		}

		// create/prepare config props needed for runtime

		configForHTML.binary = [];
		// copy and prepare resources for html 
		let resources = getConfigProp(fileConfig, config, "resources");
		if (resources && resources.length > 0) {
			for (let r = 0; r < resources.length; r++) {
				let res_path = path.join(rootPath, resources[r]);
				let res_name = path.basename(res_path);
				let res_outputPath = "assets/" + res_name;
				let res_unique_outputPath = outputPath + "assets/" + res_name;
				let res_filesize = copiedResources[res_unique_outputPath];
				if (!res_filesize) {
					// only need to copy if it has not yet been done
					if (!fs.existsSync(res_path)) {
						throw new Error("invalid filename path for resource " + res_path);
					}
					plugins.push(new CopyWebPackPlugin({
						patterns: [
							{ from: res_path, to: outputPath + "assets" },
						],
					}));
					stats = fs.statSync(res_path);
					res_filesize = stats["size"];
					copiedResources[res_unique_outputPath] = res_filesize;
				}
				configForHTML.binary.push({
					name: res_name,
					path: res_outputPath,
					size: res_filesize,
				});
			}
		}
		let assets = getConfigProp(fileConfig, config, "assets");
		if (assets && assets.length > 0) {
			for (let r = 0; r < assets.length; r++) {
				let res_path = path.join(rootPath, assets[r]);

				// extension is missing = is folder	
				if (!fs.existsSync(res_path)) {
					throw new Error("invalid filename path for asset " + res_path);
				}

				let folder = fs.lstatSync(res_path).isDirectory();
				let name = path.basename(res_path);

				plugins.push(new CopyWebPackPlugin({
					patterns: [
						{ from: res_path, to: outputPath + "assets" + (folder ? "/" + name : "")  },
					],
				}));
				
			}
		}
		
		configForHTML.binary.push({
			name: configForHTML.filename,
			path: "assets/" + configForHTML.filename + ".swf",
			size: filesize,
			resourceType: "GAME",
		});

		if (configForHTML.splash)
			configForHTML.splash = "assets/" + configForHTML.splash;

		if (configForHTML.loading)
			configForHTML.loading.image = "assets/" + configForHTML.loading.image;

		if (configForHTML.start)
			configForHTML.start.image = "assets/" + configForHTML.start.image;


		var runtimePath = "js/" + config.entryName + ".js";
		configForHTML["runtime"] = runtimePath;


		// create string for html inject (incl hack to handle functions): 

		var collectedFunctions = collectAndReplaceFunctions({}, configForHTML);
		var configStr = "\nconfig = " + JSON.stringify(configForHTML, null, 4) + ";\n";
		var jsStringForHTML = "";
		var allFunctions;
		if (Object.keys(collectedFunctions).length > 0) {
			jsStringForHTML = "\nlet allFunctions = {};\n";
			for (var key in collectedFunctions) {
				jsStringForHTML += "allFunctions['" + key + "'] = " + collectedFunctions[key].toString() + ";\n";
			}
			jsStringForHTML += configStr;
			jsStringForHTML += "\nlet connectConfigToFunctions =" + (function (obj) {
				for (var key in obj) {
					if (typeof obj[key] == "string" && obj[key].indexOf("___") === 0) {
						obj[key] = allFunctions[obj[key].replace("___", "")];
					}
					if (typeof obj[key] == "object")
						connectConfigToFunctions(obj[key]);
				}
			}).toString() + ";\n";
			jsStringForHTML += "\nconnectConfigToFunctions(config);\n";
		}
		else {
			jsStringForHTML += configStr;
		}

		// code to overwrite config by URLSearchParams
		if (config.allowURLSearchParams) {
			jsStringForHTML += `const q = new URLSearchParams(location.search);
			
			for (let key of q.keys()){ 
				let value = q.get(key);

				if (value.includes("<boolean>") || value.includes("<bool>")) {
					let valueWithoutClass = value.replace(/<boolean>/g,'').replace(/<bool>/g,'').toLowerCase();
					if (["true", "t", "1"].includes(valueWithoutClass)) config[key] = true;
					else if (["false", "f", "0"].includes(valueWithoutClass)) config[key] = false;
					else console.warn("Unable to cast URLSearchParam '" + key + "'='" + valueWithoutClass + "' as boolean");
				} else if (value.includes("<int>") || value.includes("<integer>") || value.includes("<number>")) {
					let valueWithoutClass = value.replace(/<int>/g,'').replace(/<integer>/g,'').replace(/<number>/g,'');
					config[key] = parseInt(valueWithoutClass, 10);
				} else {
					// treat this as string
					config[key] = q.get(key);
				}

				// console.log({key, value, result: config[key]}); 
			};
			`;
		}

		// add cachebuster
		if (config.cacheBuster) {
			jsStringForHTML += "for (let key in config.binary){ config.binary[key].path = config.binary[key].path+'?v='+Math.random();};\n";
		}

		
		//	copy and mod html:

		var htmlOutputPath = (config.split ? folderName + "/" : "") + (config.split ? "index.html" : folderName + ".html");
		gameURLS[fileConfig.rt_filename] = {
			path: htmlOutputPath,
			name: configForHTML.title,
		};

		var htmlSourcePath = getConfigProp(fileConfig, config, "gameTemplate");

		if (config.debugConfig) {
			console.log("### " + configForHTML.title + " CONFIG THAT WILL BE INJECTED INTO HTML");
			for (var key in configForHTML) {
				console.log("			- config." + key, configForHTML[key]);
			}
		}

		plugins.push(new CopyWebPackPlugin({
			patterns: [
				{
					from: htmlSourcePath,
					to: htmlOutputPath,
					transform: function (content, src) {
						return content.toString()
							.replace(/INSERT_TITLE/g, configForHTML.title ? configForHTML.title : "UNTITLED")
							.replace(/INSERT_SPLASHSCREEN/g, configForHTML.splash)
							.replace(/INSERT_CODE/g, jsStringForHTML);
					}
				}
			],
		}));
	};

	var swfPath, stats, filesize;
	for (var i = 0; i < config.fileconfigs.length; i++) {
		_loop_1(i);
	}


	// Generate a listing html that links to all game-htmls:
	plugins.push(new HTMLWebPackPlugin({
		title: config.rt_title,
		template: config.indexTemplate,
		filename: 'index.html',
		games: gameURLS,
		inject: false
	}));

	if (config.split) {
		// 	when webpack is finished, copy the js-folder to subfolders 
		//	this errors with dev-server, so we only use "split" in prod

		plugins.push({
			apply: function (compiler) {
				compiler.hooks.afterEmit.tap('MyPlugin', function (compilation) {
					console.log("copy build to game-folders");
					for (var i = 0; i < config.fileconfigs.length; i++) {
						copyRecursiveSync(fs, path, path.join(rootPath, "bin", "js"), path.join(rootPath, "bin", config.fileconfigs[i].rt_filename, "js"));
					}
					rimraf.sync(path.join(rootPath, "bin", "js"));
				});
			}
		});

	}

	return plugins;
}

// get config prop from file-config, or from global-config if it doesnt eists
var getConfigProp = function (fileconfig, config, name) {
	return fileconfig[name] ? fileconfig[name] : config[name];
};

// get a config for a game-file.
// this filters out all props that are not prefixed by "rt_"
// also takes care that it uses props from global-config if file-config does not provide it
// this can probably be done better and cleaner
// but for now it should do the job
var getConfig = function (fileconfig, config) {
	var newConfig = {};
	for (var key in fileconfig) {
		if (key.indexOf("rt_") == 0) {
			newConfig[key.replace("rt_", "")] = fileconfig[key];
		}
	}
	for (var key in config) {
		if (key.indexOf("rt_") == 0 && !newConfig.hasOwnProperty(key.replace("rt_", ""))) {
			newConfig[key.replace("rt_", "")] = config[key];
		}
	}

	return newConfig;
};

// collect all js-functions found in config obj and replace them with string-id
// we will inject the functions sepperatly, so we can inject config as json string
// we also inject a function that wires the collected functions back to the js-obj
// this way we can support injecting simple js-function into the html (dont use "this" in functions)
var collectAndReplaceFunctions = function (collectedFunctions, obj, path) {
	if (path === void 0) { path = ""; }
	if (path != "") { path += ""; }
	if (typeof obj === "object") {
		for (var key in obj) {
			if (typeof obj[key] === "function") {
				collectedFunctions[path + key] = obj[key];
				obj[key] = "___" + path + key;
			}
			else {
				collectedFunctions = collectAndReplaceFunctions(collectedFunctions, obj[key], path + key);
			}
		}
	}
	return collectedFunctions;
};
// used to copy the output to sub-folder when building in split mode
var copyRecursiveSync = function (fs, path, src, dest) {
	var exists = fs.existsSync(src);
	var stats = exists && fs.statSync(src);
	var isDirectory = exists && stats.isDirectory();
	if (isDirectory) {
		fs.mkdirSync(dest);
		fs.readdirSync(src).forEach(function (childItemName) {
			copyRecursiveSync(fs, path, path.join(src, childItemName), path.join(dest, childItemName));
		});
	}
	else {
		fs.copyFileSync(src, dest);
	}
};
