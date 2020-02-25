import { SWFParser } from "@awayjs/swf-viewer";
import { createSecurityDomain, AVM2LoadLibrariesFlags } from "./avmLoader";

import { initLink, Sprite, FlashSceneGraphFactory, Loader, Stage, LoaderContext, Event, URLRequest, SecurityDomain } from "@awayfl/playerglobal";
import { RequestAnimationFrame, ColorUtils, AssetEvent} from '@awayjs/core';
import { initlazy, initSystem, initializeAXBasePrototype, ActiveLoaderContext, OrphanManager } from '@awayfl/avm2';
import { FrameScriptManager } from '@awayjs/scene';

class EntryClass extends Sprite {
	constructor() {
		super();
	}
}
initSystem();
initLink();
initializeAXBasePrototype();
initlazy();
// Add the |axApply| and |axCall| methods on the function prototype so that we can treat
// Functions as AXCallables.
(<any>Function.prototype).axApply = Function.prototype.apply;
(<any>Function.prototype).axCall = Function.prototype.call;

export class Player {
	private _stage: Stage;
	private _loader: Loader;
	private _parser: SWFParser;
	private _timer: RequestAnimationFrame;
	private _time: number = 0;
	private _sec: SecurityDomain;
	private _events: any[];
	private _eventOnEnter: Event;
	private _eventFrameConstructed: Event;
	private _eventExitFrame: Event;
	private _eventRender: Event;
	private _renderStarted: boolean;
	private _debugtimer:number=0;
	constructor() {
		this._renderStarted=false;
		
		this._onLoadCompleteDelegate = (event: Event) => this.onLoadComplete(event);
		this._onAssetCompleteDelegate = (event: AssetEvent) => this._onAssetComplete(event);
	}
	public playSWF(buffer, url) {
		if (this._loader || this._parser) {
			throw "Only playing of 1 SWF file is supported at the moment";
		}
		createSecurityDomain(
			AVM2LoadLibrariesFlags.Builtin | AVM2LoadLibrariesFlags.Playerglobal
		).then((sec:SecurityDomain) => {
			console.log("builtins are loaded fine, start parsing SWF");
			this._sec = sec;

			this._eventOnEnter = new this._sec.flash.events.Event(Event.ENTER_FRAME);
			this._eventFrameConstructed = new this._sec.flash.events.Event(Event.FRAME_CONSTRUCTED);
			this._eventExitFrame = new this._sec.flash.events.Event(Event.EXIT_FRAME);
			this._eventRender = new this._sec.flash.events.Event(Event.RENDER);
			this._events = [this._eventOnEnter, this._eventExitFrame];
			this._stage = new this._sec.flash.display.Stage(null, window.innerWidth, window.innerHeight, 0xffffff, 30, true);
			this._sec.flash.display.DisplayObject.axClass._activeStage=this._stage;
			this._parser = new SWFParser(new FlashSceneGraphFactory(sec));
			this._parser._iFileName=url;
			this._loader = new this._sec.flash.display.Loader(this._parser);
			this._loader.loaderInfo.url=url;
			var loaderContext: LoaderContext = new this._sec.flash.system.LoaderContext(false, new (<any>this._sec).flash.system.ApplicationDomain());
			ActiveLoaderContext.loaderContext=loaderContext;
			this._loader.loaderInfo.addEventListener(Event.COMPLETE, this._onLoadCompleteDelegate);
			this._loader.loaderInfo.addEventListener(AssetEvent.ASSET_COMPLETE, this._onAssetCompleteDelegate);
			if(buffer){
				this._loader.loadData(buffer, loaderContext);
			}
			else{
				this._loader.load(new URLRequest(url), loaderContext);
			}
			this._stage.rendererStage.container.style.visibility="hidden";


		});
	}
	private _onAssetCompleteDelegate: (event: AssetEvent) => void;
	public _onAssetComplete(event) {
	}
	private _onLoadCompleteDelegate: (event: Event) => void;
	public onLoadComplete(event) {
		
		console.log("loaded a SWFFile", this._parser.swfFile);
		this._stage.color=ColorUtils.f32_RGBA_To_f32_ARGB(this._parser.swfFile.backgroundColor);
		this._stage.frameRate=this._parser.swfFile.frameRate;
		this._stage.stageWidth=this._parser.swfFile.bounds.width/20;
		this._stage.stageHeight=this._parser.swfFile.bounds.height/20;
		this._stage.addChild(this._loader);

		/*
		This will be needed later to support multiple scenes
		it works to create a Scene Object atm, but our MovieClip is not yet setup to handle and support it

		var sceneData=(<any>this._parser).sceneAndFrameLabelData;
		if(sceneData && sceneData.scenes && sceneData.scenes.length>0){
			var scene=new (<any>this._sec).flash.display.Scene(sceneData.scenes[0].name,[], sceneData.scenes[0].offset, 1);
			scene.axInitializer();
		}
		*/

		// loading is finished, start the main_loop:
		this._timer = new RequestAnimationFrame(this.main_loop, this);
		this._timer.start();
	}
	/**
	 * Main loop
	 */
	private main_loop(dt: number) {
		var frameMarker: number = Math.floor(1000 / this._stage.frameRate);
		this._time += Math.min(dt, frameMarker);

		if (this._time >= frameMarker || !this._renderStarted) {
			
			this._time -= frameMarker;

			// 	broadcast ENTER_FRAME event
			//	i think this should really be done after the stage has advanced, but it causes a error in BIC
			this._stage.dispatchStaticBroadCastEvent(Event.ENTER_FRAME);

			// 	advance the stage - this updates the timeline
			//	objects get removed, created and updated - framescripts get queued
			//	as3 constructors for the adpaters are not yet run
			//	ADD events are queued on the objects, because they need to run after the constructors
			this._stage.advanceFrame(this._events);
			OrphanManager.updateOrphans(this._events);
			
			
			// run all as3 constructors
			FrameScriptManager.execute_as3_constructors();
			
			// 	ADD events events have been queued on the objects before their constructor was run
			//	dispatch them now (this is recursivly)
			this._stage.dispatchQueuedEvents();

			// broadcast FRAME_CONSTRUCTED event to all objects
			this._stage.dispatchStaticBroadCastEvent(Event.FRAME_CONSTRUCTED);

			// run all queued framescripts
			FrameScriptManager.execute_queue();
			
			/*var displayGraph={};
			this._stage.debugDisplayGraph(displayGraph);
			console.log("SceneGraph frame :", displayGraph);*/
			
			// execute queued scripts
			
			// render
			this._stage.render();
			if(!this._renderStarted){
				if(window["hidePokiProgressBar"])
					window["hidePokiProgressBar"]();
				this._stage.rendererStage.container.style.visibility="visible";
			}
			this._renderStarted=true;
		}
	}
}
