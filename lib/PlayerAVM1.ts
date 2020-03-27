import { SWFParser, StageScaleMode, StageAlign, MovieClipSoundsManager, MovieClipSoundStream } from "@awayfl/swf-loader";
import { AVMAwayStage, AVM1SceneGraphFactory, AVM1Globals, TraceLevel, AVM1ContextImpl, SecurityDomain, LoaderInfo, AVM1MovieClip } from "@awayfl/avm1";
import { AssetLibrary, AudioManager, AssetEvent, LoaderEvent, URLLoaderEvent, URLRequest, ColorUtils, IAsset } from "@awayjs/core";
import { MovieClip, Font, DefaultFontManager, FrameScriptManager } from "@awayjs/scene";



export class PlayerAVM1 {
	private _stage: AVMAwayStage;
	private _avm1SceneGraphFactory: AVM1SceneGraphFactory;
	private _parser: SWFParser;
	private _skipFrames: number=0;
	private _skipFramesCallback: Function;
	private showAdOnFrame: number=-1;
	private _fontsLoadedCallback:Function=null;

	constructor() {
		
		// create the AwayStage for AVM1
		this._stage = new AVMAwayStage(window.innerWidth / 2, window.innerHeight / 2, 0xffffff, 24, null, window.innerWidth / 2, window.innerHeight / 2, false);
		this._stage.scene.mouseManager._stage=this._stage;
		this._stage.rendererStageContainer.style.visibility="hidden";
		var loaderInfo=new LoaderInfo();
		this._avm1SceneGraphFactory = new AVM1SceneGraphFactory(new AVM1ContextImpl(loaderInfo));
		this._avm1SceneGraphFactory.avm1Context.sec = new SecurityDomain();
		this._avm1SceneGraphFactory.avm1Context.setStage(this._stage, document);
		AVM1Globals.tracelevel=TraceLevel.ALL;
		AVM1Globals._scenegraphFactory=this._avm1SceneGraphFactory;
		AssetLibrary.enableParser(SWFParser);
		this._onAssetCompleteDelegate = (event: AssetEvent) => this._onAssetComplete(event);
		this._onLoadCompleteDelegate = (event: LoaderEvent) => this.onLoadComplete(event);
		this._onLoadErrorDelegate = (event: URLLoaderEvent) => this._onLoadError(event);
		this._onAssetCompleteFontsDelegate = (event: AssetEvent) => this._onAssetFontsComplete(event);
		this._onLoadCompleteFontsDelegate = (event: LoaderEvent) => this.onLoadFontsComplete(event);
		this._onLoadErrorFontsDelegate = (event: URLLoaderEvent) => this._onLoadFontsError(event);
		window.addEventListener("resize", (e)=>this._stage.resizeCallback(e));
	}
	public registerCustomExitFrameCallback(callback:Function):any {
		return this._stage.registerCustomExitFrameCallback(callback);
	}
	public screenshot(callback:Function):any {
		return this._stage.screenshot(callback);
	}
	public get avm1SceneGraphFactory():any {
		return this._avm1SceneGraphFactory;
	}
	public preloadFonts(url:string, onFinished:Function) {
		this._fontsLoadedCallback=onFinished;
		if(!this._parser){
			this._parser=new SWFParser(this._avm1SceneGraphFactory);
			this._parser._iFileName=url;
		}
		AssetLibrary.addEventListener(AssetEvent.ASSET_COMPLETE, this._onAssetCompleteFontsDelegate);
		AssetLibrary.addEventListener(LoaderEvent.LOAD_COMPLETE, this._onLoadCompleteFontsDelegate);
		AssetLibrary.addEventListener(URLLoaderEvent.LOAD_ERROR, this._onLoadErrorFontsDelegate);
		AssetLibrary.load(new URLRequest(url), null, null, this._parser);

	}
	public playSWF(buffer, url, skipFramesOnScene=0, skipFramesCallback=null) {
		this._skipFrames=skipFramesOnScene;
		this._skipFramesCallback=skipFramesCallback;
		AudioManager.setVolume(1);
		(<AVM1ContextImpl>this._avm1SceneGraphFactory.avm1Context).executionProhibited=false;
		if(this._parser){
			this._parser.dispose();
		}
		this._parser=new SWFParser(this._avm1SceneGraphFactory);
		this._parser._iFileName=url;
		
		AssetLibrary.addEventListener(AssetEvent.ASSET_COMPLETE, this._onAssetCompleteDelegate);
		AssetLibrary.addEventListener(LoaderEvent.LOAD_COMPLETE, this._onLoadCompleteDelegate);
		AssetLibrary.addEventListener(URLLoaderEvent.LOAD_ERROR, this._onLoadErrorDelegate);
		AssetLibrary.loadData(buffer, null, null, this._parser);
	}
	private _onLoadErrorFontsDelegate: (event: URLLoaderEvent) => void;
	public _onLoadFontsError(event) {
		console.log("error preloading fonts", event);
		AssetLibrary.removeEventListener(AssetEvent.ASSET_COMPLETE, this._onAssetCompleteFontsDelegate);
		AssetLibrary.removeEventListener(LoaderEvent.LOAD_COMPLETE, this._onLoadCompleteFontsDelegate);
		AssetLibrary.removeEventListener(URLLoaderEvent.LOAD_ERROR, this._onLoadErrorFontsDelegate);
	}
	private _onAssetCompleteFontsDelegate: (event: AssetEvent) => void;
	public _onAssetFontsComplete(event) {		
		var asset: IAsset = event.asset;
		if (asset.isAsset(Font)) {
			DefaultFontManager.deviceFont=<Font>asset;
			console.log("font loaded", asset);
		}				
	}
	private _onLoadCompleteFontsDelegate: (event: LoaderEvent) => void;
	public onLoadFontsComplete(event) {
		
		AssetLibrary.removeEventListener(AssetEvent.ASSET_COMPLETE, this._onAssetCompleteFontsDelegate);
		AssetLibrary.removeEventListener(LoaderEvent.LOAD_COMPLETE, this._onLoadCompleteFontsDelegate);
		AssetLibrary.removeEventListener(URLLoaderEvent.LOAD_ERROR, this._onLoadErrorFontsDelegate);
		if(this._fontsLoadedCallback){
			this._fontsLoadedCallback();
		}
	}
	private _onLoadErrorDelegate: (event: URLLoaderEvent) => void;
	public _onLoadError(event) {
	}
	private _onAssetCompleteDelegate: (event: AssetEvent) => void;
	public _onAssetComplete(event) {		
		var asset: IAsset = event.asset;
		if (asset.isAsset(MovieClip)) {
			if ((<any>asset).isAVMScene) {				
				if(this._stage)
					this._stage.getLayer(0).addChild(<MovieClip>asset);
					
				(<AVM1MovieClip>(<MovieClip>asset).adapter).initAdapter();
			}
		}				
	}
	private _onLoadCompleteDelegate: (event: LoaderEvent) => void;
	public onLoadComplete(event) {
		
		//console.log("loaded a SWFFile", this._parser.swfFile);
		console.debug("FP-VERSION", this._parser.swfFile.fpVersion);
		this._stage.color=ColorUtils.f32_RGBA_To_f32_ARGB(this._parser.swfFile.backgroundColor);
		this._stage.frameRate=this._parser.swfFile.frameRate;
		MovieClipSoundStream.frameRate=this._parser.swfFile.frameRate;
		this._stage.stageWidth=this._parser.swfFile.bounds.width/20;
		this._stage.stageHeight=this._parser.swfFile.bounds.height/20;
		this._avm1SceneGraphFactory.avm1Context.swfVersion=this._parser.swfFile.swfVersion;
		this._stage.scaleMode=StageScaleMode.SHOW_ALL;
		this._stage.align=StageAlign.TOP;
		this._stage && this._stage.resizeCallback();
		AssetLibrary.removeEventListener(AssetEvent.ASSET_COMPLETE, this._onAssetCompleteDelegate);
		AssetLibrary.removeEventListener(LoaderEvent.LOAD_COMPLETE, this._onLoadCompleteDelegate);
		AssetLibrary.removeEventListener(URLLoaderEvent.LOAD_ERROR, this._onLoadErrorDelegate);
		
		if(window["hidePokiProgressBar"])
			window["hidePokiProgressBar"]();
			
		if(this._skipFrames>0){
			FrameScriptManager.execute_queue();
			if(this._skipFramesCallback){
				AudioManager.setVolume(0);
				this._skipFramesCallback(()=>{
					AudioManager.setVolume(1);
					(<MovieClip>this._stage.getLayer(0).getChildAt(0)).currentFrameIndex=this._skipFrames;
					(<MovieClip>this._stage.getLayer(0).getChildAt(0)).play();
				})
			}
			else{
				(<MovieClip>this._stage.getLayer(0).getChildAt(0)).currentFrameIndex=this._skipFrames;
				(<MovieClip>this._stage.getLayer(0).getChildAt(0)).play();
			}
		}
		this._stage.rendererStageContainer.style.visibility="visible";
	}
}
