import { AVMStage, AVMEvent, AVMVERSION, registerDebugMethod, release } from "@awayfl/swf-loader";

import { AVM1Handler } from '@awayfl/avm1';
import { AVM2Handler } from '@awayfl/avm2';
import { PlayerGlobal } from "@awayfl/playerglobal";


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

export class AVMPlayer extends AVMStage {
	constructor() {
		super();
		this.registerAVMStageHandler(new AVM1Handler());
		this.registerAVMStageHandler(new AVM2Handler(new PlayerGlobal()));
		this.addEventListener(AVMEvent.AVM_COMPLETE, (event: AVMEvent) => this.onAVMAvailable(event));

		
		// export player api
		if(!release) {
			registerDebugMethod(this._getInfo.bind(this), { 
				name: "getInfo", 
				description:"Get file info for app", 
				declaration: [{name:"return", type:"object"}] 
			});
		}
	}

	protected onAVMAvailable(event: AVMEvent) {
		if (event.avmVersion == AVMVERSION.AVM1) {
			console.log("AVM1 has init");
		}
		else if (event.avmVersion == AVMVERSION.AVM2) {
			console.log("AVM2 has init");
		}

	}

	private _getInfo() {
		const thisAny = <any>this;

		const avm = this._avmHandler.avmVersion;
		const { 
			swfVersion,
			fpVersion,
			frameCount,
			frameRate,
			compression,
			bytesTotal
		} = thisAny._swfFile;

		let path: string = (<any>this._gameConfig).binary.filter(({resourceType}) => resourceType === 'GAME')[0]?.path;

		if(path && path.indexOf('?') > -1) {
			path = path.substring(0, path.indexOf('?'));
		}

		return {
			file: {
				name: (<any>this._gameConfig).title,
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
			config: fullSerializer(this._gameConfig)
		}
	}
}

