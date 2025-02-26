import "./nape.max.js";

//@ts-ignore
const nape = self.nape;
//@ts-ignore
const zpp_nape = self.zpp_nape;

const _PolygonConstructor = nape.shape.Polygon;
const _Body = nape.phys.Body;
const _Interactor = nape.phys.Interactor;


//add toPoint
//@ts-ignore
nape.geom.Vec2.prototype.toPoint = function(output:any) {
    output.x = this.x;
    output.y = this.y;
    return output;
}

nape.geom.GeomPoly = class _NewGeomPoly extends nape.geom.GeomPoly {
    constructor(v: any) {
        //@ts-ignore
        if (v?._buffer)
            v = v._buffer;

        super(v);
    }
}

//@ts-ignore
nape.callbacks.InteractionCallback.axIsType = function(x: any): boolean {
    return x instanceof this;
}
 

// nape.shape.Polygon = class _NewPolygon extends nape.shape.Polygon {
//     constructor(v: any, m: any, f: any) {
//         //@ts-ignore
//         v = v.value ? v.value: v;
//         super(v, m, f);
//     }
// }

// const _add = zpp_nape.util.ZNPList_ZPP_InteractionListener.prototype.add;
// zpp_nape.util.ZNPList_ZPP_InteractionListener.prototype.add = function(o: any): any {
//     let l = o.handleri;
//     // ASFunction closure
//     if( typeof l.value === 'function') {
//         o.handleri = l.value.bind(l.receiver);
//     }
//     return _add.call(this, o) as any;
// }
// // restore bechaviour of version 
// // https://github.com/deltaluca/nape/commit/18ffe75f1e943023f2608dc220660aeca2504898
// // @ts-ignore
// nape.phys.Body.prototype.worldToLocal = nape.phys.Body.prototype.worldPointToLocal;
// // @ts-ignore
// nape.phys.Body.prototype.localToWorld = nape.phys.Body.prototype.localPointToWorld;
// //@ts-ignore
// nape.phys.Body.prototype.applyWorldImpulse = nape.phys.Body.prototype.applyImpulse;

// //@ts-ignore
// nape.phys.Body.prototype.applyLocalForce = function(vector: nape.geom.Vec2, pos) {
//     const iter = <nape.phys.Body> this;
//     const dt = iter.space.zpp_inner.pre_dt;
//     nape.phys.Body.prototype.applyImpulse.call(iter, vector.mul(dt, true), pos);
// }

// //@ts-ignore
// nape.phys.Body.prototype.applyLocalImpulse = nape.phys.Body.prototype.applyImpulse;

// // hack user data, in newes nape it a object, need inser fields to it
// nape.phys.Body = class PathedBody extends _Body {
//     constructor(type,position) {
//         super(type, position);

//         Object.defineProperty(this, 'userData', {
//             //@ts-ignore
//             get: _Interactor.prototype.get_userData,
//             set: function(v) {
//                 this.zpp_inner_i.userData = v;
//             }
//         })
//     }
// }

// // restore
// // https://github.com/deltaluca/nape/blob/0759bf48e2274d0f024bf491c2bf06d66d5060d6/cx-src/nape/geom/GeomPoly.cx#L216
// // @ts-ignore
// nape.geom.GeomPoly.prototype.simple_decomposition = nape.geom.GeomPoly.prototype.simpleDecomposition;
// // @ts-ignore
// nape.geom.GeomPoly.prototype.convex_decomposition = nape.geom.GeomPoly.prototype.convexDecomposition;


export default { nape };