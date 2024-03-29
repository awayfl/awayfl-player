import { BitmapImage2D, ImageUtils } from "@awayjs/stage";

console.debug("AwayFL-Player - 0.2.43");

export {AVMPlayer} from "./lib/AVMPlayer";
export {AVM1Player} from "./lib/AVM1Player";
export {AVM2Player} from "./lib/AVM2Player";
export {LoaderEvent} from "@awayjs/core";
export {EventBase} from "@awayjs/core";
export {StageManager} from "@awayjs/stage";
export {Settings as AVM2Settings} from "@awayfl/avm2";
export {PlayerGlobal} from "@awayfl/playerglobal";

ImageUtils.registerDefaults(
	() => new BitmapImage2D(1, 1, true, 0x0),
	null,
	null,
	null,
);

export * as Box2D from "./lib/Box2D";
export * as Box2DLegacy from "./lib/Box2Dold";
export * as Box2DRaycast from "./lib/Box2Dold_Raycast";