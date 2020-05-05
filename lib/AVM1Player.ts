import { AVMStage } from "@awayfl/swf-loader";
import { AVM1Handler } from '@awayfl/avm1';


export class AVM1Player extends AVMStage {
	constructor(gameConfig) {
		super(gameConfig);
		this.registerAVMStageHandler(new AVM1Handler());
	}
}