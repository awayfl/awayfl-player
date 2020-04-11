import { AVMStage, AVMEvent, AVMVERSION } from "@awayfl/swf-loader";

import { AVM1Handler } from '@awayfl/avm1';
import { AVM2Handler } from '@awayfl/avm2';
import { PlayerGlobal } from "@awayfl/playerglobal";



export class AVMPlayer extends AVMStage {
	constructor() {
		super();
		this.registerAVMStageHandler(new AVM1Handler());
		this.registerAVMStageHandler(new AVM2Handler(new PlayerGlobal()));
		this.addEventListener(AVMEvent.AVM_COMPLETE, (event: AVMEvent) => this.onAVMAvailable(event));
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

