import { AVMStage, AVMEvent, AVMVERSION, release } from "@awayfl/swf-loader";
import { AVMDebug } from "./AVMDebugInterface";

import { AVM1Handler } from '@awayfl/avm1';
import { AVM2Handler, extClasses } from '@awayfl/avm2';
import { PlayerGlobal } from "@awayfl/playerglobal";


export class AVMPlayer extends AVMStage {
	private _debug: AVMDebug;
	constructor(gameConfig) {
		super(gameConfig);

		if(gameConfig.externalLib) {
			extClasses.lib = gameConfig.externalLib;
		}

		this.registerAVMStageHandler(new AVM1Handler());
		this.registerAVMStageHandler(new AVM2Handler(new PlayerGlobal()));
		this.addEventListener(AVMEvent.AVM_COMPLETE, (event: AVMEvent) => this.onAVMAvailable(event));

		// export player api
		//if(!release) {
		this._debug = new AVMDebug(this);
		//}
	}

	protected onAVMAvailable(event: AVMEvent) {
		if(this._debug) {
			this._debug.onAvmInit(event.avmVersion === AVMVERSION.AVM1 ? 1 : 2);
		}
	}
}

