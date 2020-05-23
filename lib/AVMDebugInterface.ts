
import { AVMStage, AVMEvent, AVMVERSION, registerDebugMethod, release } from "@awayfl/swf-loader";

function fullSerializer(obj: any) {
	const clone = Object.assign({}, obj);

	Object.keys(clone).forEach((key)=>{
		if(typeof clone[key] === 'object') {
			clone[key] = fullSerializer(clone[key]);
		} else if(typeof clone[key] === 'function') {
			// replace func with it string representation
			clone[key] = clone[key].toString();
		}
	});

	return clone;
}

export class AVMDebug {
    constructor(public player: AVMStage) {
        registerDebugMethod(this._getInfo.bind(this), { 
            name: "getInfo", 
            description:"Get file info for app", 
            declaration: [{name:"return", type:"object"}] 
        });
    }
    
	private _getInfo() {
		const player = <any>this.player;

		const avm = player._avmHandler.avmVersion;
		const { 
			swfVersion,
			fpVersion,
			frameCount,
			frameRate,
			compression,
			bytesTotal
		} = player._swfFile;

		let path: string = (<any>player._gameConfig).binary.filter(({resourceType}) => resourceType === 'GAME')[0]?.path;

		if(path && path.indexOf('?') > -1) {
			path = path.substring(0, path.indexOf('?'));
		}

		return {
			file: {
				name: (<any>player._gameConfig).title,
				path: path,
				size: bytesTotal
			},
			runtime: {
				swfVersion, 
				fpVersion, 
				frameCount, 
				frameRate, 
				compression,
				avm
			},
			config: fullSerializer(player._gameConfig)
		}
	}
}
	