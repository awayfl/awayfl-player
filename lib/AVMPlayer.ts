import { AVMStage, AVMEvent, AVMVERSION,release } from "@awayfl/swf-loader";
import { AVMDebug } from "./AVMDebugInterface";

import { AVM1Handler } from '@awayfl/avm1';
import { AVM2Handler } from '@awayfl/avm2';
import { PlayerGlobal } from "@awayfl/playerglobal";


export class AVMPlayer extends AVMStage {
	constructor(gameConfig) {
		super(gameConfig);
		this.registerAVMStageHandler(new AVM1Handler());
		this.registerAVMStageHandler(new AVM2Handler(new PlayerGlobal(), gameConfig.forceJIT));
		this.addEventListener(AVMEvent.AVM_COMPLETE, (event: AVMEvent) => this.onAVMAvailable(event));

		// export player api
		if(!release) {
			new AVMDebug(this);
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
}

