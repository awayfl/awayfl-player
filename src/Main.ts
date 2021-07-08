import { AVMPlayer } from "../index"

class Main extends AVMPlayer {
	constructor(gameConfig: any) {
		super(gameConfig);

        // LoaderInfo.DefaultLocation="/";
        // gameConfig.redirects = [{
        //     test: /img/,
        //     resolve: (url) => `./assets/${url.replace(/\/\//g,'')}`
		// },{
        //     test: /media/,
        //     resolve: (url) => `./assets/${url.replace(/\/\//g,'')}`
		// }];
    }
};

window["AVMPlayerClass"] = Main;