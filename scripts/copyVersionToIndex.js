"use strict";
exports.__esModule = true;
var fs = require("fs");
var path = require("path");


// read in the ts-file at filePath

// use regex to find a console log for printing the version and update it for the new version

// update ts-file at filePath with the new content

//console.log(process.env.npm_package_version);
//console.log(process.env.INIT_CWD);
const args = process.argv.slice(2);
if (!args[0]) {
	console.log("copyVersionToIndex - no path was provided")
}
let filePath = path.join(process.env.INIT_CWD, args[0]);

console.log("update ", filePath, " with version:", process.env.npm_package_version);

fs.readFile(filePath, 'utf8', function (err, data) {
	if (err) throw err;
	var re = /(.*[a-zA-Z0-9]\s\-\s)(.*)(\"\)\;.*)/;
	//console.log("before", data)
	data = data.replace(re, "$1" + process.env.npm_package_version + "$3");//#BUILD_VIA_NPM_VERSION_PATCH_TO_DISPLAY_VERSION_HERE#", process.env.npm_package_version);
	//console.log("after", data)
	fs.writeFile(filePath, data, function (err) {
		if (err) throw err;
		console.log("Updated ", filePath, " with inserted version ", process.env.npm_package_version);
	});
});
