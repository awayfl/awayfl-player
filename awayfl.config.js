const path = require('path');

module.exports = {


	// all props prefixed with "rt_" will be added to config that is inserted in html

	//------------------------------------------------------
	// global config - same for all swf-files:
	//------------------------------------------------------

	debugConfig: false,			//	log config in build process
	rt_showFPS: false,			//	show fps display - always false in prod
	cacheBuster: true,			//	add cachebuster to urls - always false in prod
	allowURLSearchParams: true, //	allow changing config via url-params - always false in prod
	split: true, 				//	create own folder for each file - only available in prod

	entryName: "Main", 		//	name of webpack-entry - must be set for each config (use package.main ?)
	entryPath: "./src/Main.ts", 		//	path to webpack-entry - must be set for each config (use package.main ?)

	buildinsPath: path.join(__dirname, "builtins"), 	//	path to buildins - must be set when amv2 will be used
	indexTemplate: path.join(__dirname, "template", "index_template.html"), 	//	path to game-html template - must be set
	gameTemplate: path.join(__dirname, "template", "game_template.html"), 	//	path to index-html template - must be set
	loaderTemplate: path.join(__dirname, "template", "loader.js"),	//	path to loader.js - must be set

	//-------------------------------------------------------------------------
	// default config for this game - can be overwritten for every file-config:
	//-------------------------------------------------------------------------

	rt_debug: true, 	//	disable JS blobind - always false in prod
	rt_title: "Main", 		//	title of game - should be overwritten for each file-config, but also available for index
	rt_filename: "",	//	filename of game - no extension - must be set for each config
	rt_splash: "todo.jpg",		//	path to splash-image - with extension
	rt_start: null,		//	path to start-image - with extension - optional - if present, loader wait for user input to start the game
	rt_width: 1024,		//	width of preloader screen (todo: grab this from splashimage ?)
	rt_height: 768,		//	height of preloader screen (todo: grab this from splashimage ?)	
	rt_x: 0,		// x offset of stage (either absolute px value, or string with percentage of window.innerWidth (0-100))
	rt_y: 0,		// y offset of stage (either absolute px value, or string with percentage of window.innerHeight (0-100))
	rt_w: "100%",		// width of stage (either absolute px value, or string with percentage of window.innerWidth (0-100))
	rt_h: "100%",		// height of stage (either absolute px value, or string with percentage of window.innerHeight (0-100))

	rt_stageScaleMode:null, // allowed values: EXACT_FIT noBorder noScale showAll
	rt_stageAlign:null, // allowed values: B BL BR L R T TL TR
	
	rt_progressParserWeigth: 1,	// 	weight of parser in reporter - can be ommited or set to 0 aswell

	// properties for progress bar
	rt_progress: {
		direction: "lr", //	lr, td
		back: "#130d02", // #000
		line: "#f29f01", // "#00ff00",
		rect: [0.25, 0.77, 0.5, 0.01], // values are percentage of width / height
	},

	rt_box2dVersion: 'none', // box2d version: none, legacy, new, custom (external implementation)
	// list of file-configs. 
	// each file-config is a object that must provide:
	//	- rt_title 
	//	- rt_filename (no extension)
	// it can overwrite other config props aswell
	fileconfigs: [
		{
			rt_title: "Bacon_Ipsem",
			rt_filename: "Bacon_Ipsem",
			rt_stageScaleMode: "showAll",
		},
		{
			rt_showFPS: true,
			rt_title: "BasicAS3Tests_FP30",
			rt_filename: "BasicAS3Tests_FP30",
			rt_stageScaleMode: "showAll",
		},
		{
			rt_title: "text_test",
			rt_filename: "text_test",
			rt_stageScaleMode: "showAll",
		}
	],

	resources: ["template/fonts.swf"],	// list of urls to preload (fonts) - relative to project folder

	assets:[

	],
};
