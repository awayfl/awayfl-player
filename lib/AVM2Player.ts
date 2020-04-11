import { AVMStage } from "@awayfl/swf-loader";
import { AVM2Handler } from "@awayfl/avm2";
import { PlayerGlobal } from '@awayfl/playerglobal';


export class AVM2Player extends AVMStage {
	constructor() {
		super();
		this.registerAVMStageHandler(new AVM2Handler(new PlayerGlobal()));
	}
}

