import assert from 'node:assert/strict';
import setup from './dh_component.mjs';
const noop=()=>{};
const allRaster=[];
function canvas(){
  const handlers=new Map();
  const ctx=new Proxy({
    createImageData:(w,h)=>({data:new Uint8ClampedArray(w*h*4),width:w,height:h}),
    putImageData:(image)=>allRaster.push(new Uint8ClampedArray(image.data)),
  },{get(obj,key){return key in obj?obj[key]:noop;},set(obj,key,value){obj[key]=value;return true;}});
  return {width:600,height:470,handlers,
    getBoundingClientRect(){return{width:600,height:470,left:0,top:0};},
    getContext(){return ctx;},setPointerCapture:noop,
    addEventListener(t,fn){handlers.set(t,fn);},removeEventListener(t){handlers.delete(t);}};
}
globalThis.window={devicePixelRatio:1};
globalThis.document={createElement:canvas};
globalThis.ResizeObserver=class{observe(){}disconnect(){}};
globalThis.requestAnimationFrame=(fn)=>{fn();return null;};
globalThis.cancelAnimationFrame=noop;
function run(first){
  const w=canvas(),c=canvas(),root={__rmCleanup:null,
    querySelector(q){return ({'.rm-grid':{getBoundingClientRect:()=>({width:600})},'.rm-workspace':w,'.rm-cspace':c})[q];}};
  const events=[];
  const initial=allRaster.length;
  setup({parentElement:root,data:{joints:[
    {type:'R',a:1,theta:0,s:0,stroke:3},{type:'R',a:1,theta:0,s:0,stroke:3}],
    obstacles:[first,{x:20,y:20,shape:'Circle',radius:.125,width:.5,height:.5,rotation_deg:0}],
    axis_x:0,axis_y:1,workspace_limit:2.6,quality:1,
    show_angle_arcs:true,show_labels:true,link_half_width:.125},
    setStateValue:(key,obj)=>{if(key==='event')events.push(obj);}});
  const pixels=allRaster.at(-1);
  assert.ok(allRaster.length>initial && pixels.length===32*32*4);
  const blocked=pixels.reduce((cnt,_,idx)=>cnt+(idx%4===0 && pixels[idx]===51 ? 1:0),0);
  return {blocked,w,root,events};
}
const ob=(shape,x=1.0,y=.6,radius=.15,width=.3,height=.3,rotation_deg=0)=>
  ({shape,x,y,radius,width,height,rotation_deg});
const small=run(ob('Circle'));
const large=run(ob('Circle',1,.6,.5));
assert.ok(large.blocked>small.blocked,`circle radius did not increase blocked cells (${small.blocked} => ${large.blocked})`);
small.root.__rmCleanup();large.root.__rmCleanup();
const smallSquare=run(ob('Square',1,.6,.15,.2,.3));
const largeSquare=run(ob('Square',1,.6,.15,1.1,.3));
assert.ok(largeSquare.blocked>smallSquare.blocked);
smallSquare.root.__rmCleanup();largeSquare.root.__rmCleanup();
const triangle=run(ob('Triangle',1,.6,.15,.8,.6,25));
const rectangle=run(ob('Rectangle',1,.6,.15,.8,.6,25));
assert.ok(triangle.blocked>0 && rectangle.blocked>0);
triangle.root.__rmCleanup();rectangle.root.__rmCleanup();
// Drag an actual rectangle by its center; emit position, preserve chosen shape.
const obj=run(ob('Rectangle',1,.6,.15,.8,.6,25));
const pixel=(x,y)=>({x:120+(x+2.6)/(5.2)*396,y:28+(2.6-y)/(5.2)*396});
const ev=(name,p)=>obj.w.handlers.get(name)({clientX:p.x,clientY:p.y,pointerId:8,preventDefault:noop});
ev('pointerdown',pixel(1,.6));ev('pointermove',pixel(1.8,-.4));ev('pointerup',pixel(1.8,-.4));
assert.equal(obj.events.at(-1)?.kind,'obstacle');
assert.ok(Math.abs(obj.events.at(-1).obstacles[0][0]-1.8)<1e-6);
assert.ok(Math.abs(obj.events.at(-1).obstacles[0][1]+.4)<1e-6);
obj.root.__rmCleanup();
console.log('PASS: circle radius, square size, rectangle/triangle rotation geometry, drag-to-move polygon');
