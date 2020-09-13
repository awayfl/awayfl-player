
import { AVMStage,  registerDebugMethod } from "@awayfl/swf-loader";
import { DisplayObject } from '@awayjs/scene';
import { PickGroup } from "@awayjs/view";
import { Point } from '@awayjs/core';

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

const OBJECT_FIELDS = ['id','visible', 'index', 'assetType:type', 'name'];

export class AVMDebug {
    constructor(public player: AVMStage) {
        
        registerDebugMethod(this._dirObjectByIds.bind(this), { 
            name: "dirObjectByIds", 
            description:"Export selected object to console", 
            declaration: [{name: 'ids', type: "object"}] 
        });

        registerDebugMethod(this._applyPropsByIds.bind(this), { 
            name: "applyPropsByIds", 
            description:"Apply propertyes by node ids", 
            declaration: [{name: 'ids', type: "object"}, {name:'object', type:'object'}] 
        });

        registerDebugMethod(this._removeObjectByIds.bind(this), { 
            name: "removeObjectByIds", 
            description:"Remove object from sceen tree", 
            declaration: [{name: 'ids', type: "object"}] 
        });

        registerDebugMethod(this._getInfo.bind(this), { 
            name: "getInfo", 
            description:"Get file info for app", 
            declaration: [{name:"return", type:"object"}] 
        });
        
        registerDebugMethod(this._getSceneTree.bind(this), { 
            name: "getNodeTree", 
            description:"Get sceen tree of app", 
            declaration: [
                {name:"return", type:"object"}, 
                {name:"flat", type:"boolean"}, 
                {name:"from", type:"number"},
                {name:"rect", type:"object"}
            ] 
        });

        registerDebugMethod(this._getStageCanvas.bind(this), { 
            name: "getStageCanvas", 
            description:"Get canvas attahed to stage", 
            declaration: [] 
        });

        //@ts-ignore
        window._AWAY_DEBUG_PLAYER_ = this;
    }

    private _selectNode(ids: number[]): DisplayObject {
        let node = this.player as any;
        
        for(let i of ids) {
            node = node._children.find((e) => e.id === i);
            if(!node) {
                break;
            }
        }

        if(!node){
            throw new Error("Node not found");
        }

        return node;
    }

    private _getStageCanvas() {
        return this.player.scene.view.stage.container;
    }

    private _dirObjectByIds(ids: number[]){    
        console.dir(this._selectNode(ids));
    }

    private _getNodeBounds(node: DisplayObject) {
        const view = this.player.scene.view;
        const stage = view.stage;

        const box = PickGroup.getInstance(view).getBoundsPicker(node.partition).getBoxBounds(this.player);
        if (!box) {
            return null;
        }

        const sx = view.width / this.player.stageWidth;
        const sy = view.height / this.player.stageHeight;

        //console.log("DisplayObject:getRect not yet implemented");FromBounds
        return  {
            x: box.x * sx, 
            y: box.y * sy, 
            width: box.width * sx, 
            height: box.height * sy
        };
    }

    private _traverse(node: any, req = false, rect = false) {

        const ret =  {
            parentId: node.parent ? node.parent.id : -1,
            children: null,
            rect: null,
        }

        for(let name of OBJECT_FIELDS) {
            const sub = name.split(":");
            if(sub.length > 1) {
                ret[sub[1]] = node[sub[0]];
            } else{
                ret[name] = node[name];
            }
        }

        ret["globalVisible"] = 
            node.parent ? (node.parent.visible && node.visible) : node.visible;

        if(rect) {
            ret.rect = this._getNodeBounds(node)
        }

        if(req) {
            ret.children = node._children.map((e) => this._traverse(e, req, rect));
        }

        return ret;
    }

    private _removeObjectByIds(ids: number[]) {
        const node = this._selectNode(ids);

        node.parent.removeChild(node);
    }

    private _applyPropsByIds(ids: number[], object: any) {
        const node = this._selectNode(ids);

        Object.assign(node, object);
    }

    private _getSceneTree(flat = false, from = 0, rect = false) {
        const tree = [];
        const q: any[] = this.player._children.slice();

        while(true) {
            const node = q.pop();

            if(!node) {
                break;
            }

            tree.push(this._traverse(node, !flat, rect));
            
            if(flat){
                q.push.apply(q, node._children.reverse());
            }
        }

        return tree;
    }

	private _getInfo() {
		const player = <any>this.player;

		const avm = player._avmHandler.avmVersion;
		const { 
			swfVersion,
			fpVersion,
			frameCount,
			frameRate,
			compression,
			bytesTotal
		} = player._swfFile;

		let path: string = (<any>player._gameConfig).binary.filter(({resourceType}) => resourceType === 'GAME')[0]?.path;

		if(path && path.indexOf('?') > -1) {
			path = path.substring(0, path.indexOf('?'));
		}

		return {
			file: {
				name: (<any>player._gameConfig).title,
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
			config: fullSerializer(player._gameConfig)
		}
	}
}
	