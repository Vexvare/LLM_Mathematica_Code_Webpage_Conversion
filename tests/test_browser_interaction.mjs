/* Run: node tests/test_browser_interaction.mjs
   Lightweight Canvas/DOM simulation, including pointer gestures and emitted state. */
import assert from 'node:assert/strict';
import setup from './dh_component.mjs';
const noop=()=>{};
function canvas(){
  const handlers=new Map();
  const ctx=new Proxy({createImageData:(w,h)=>({data:new Uint8ClampedArray(w*h*4),width:w,height:h})},
      {get(obj,key){return key in obj?obj[key]:noop;},set(obj,key,value){obj[key]=value;return true;}});
  return {width:600,height:470,handlers,ctx,
    getBoundingClientRect(){return{width:600,height:470,left:0,top:0};},
    getContext(){return ctx;},setPointerCapture:noop,
    addEventListener(t,fn){handlers.set(t,fn);},removeEventListener(t){handlers.delete(t);}};
}
globalThis.window={devicePixelRatio:1};
globalThis.document={createElement:canvas};
globalThis.ResizeObserver=class{observe(){}disconnect(){}};
globalThis.requestAnimationFrame=(fn)=>{fn();return null;};
globalThis.cancelAnimationFrame=noop;
const close=(a,b,tol=.03)=>assert.ok(Math.abs(a-b)<=tol,`${a} != ${b}`);
function run(data){
  const w=canvas(),c=canvas(),root={__rmCleanup:null,
    querySelector(q){return ({'.rm-grid':{getBoundingClientRect:()=>({width:600})},'.rm-workspace':w,'.rm-cspace':c})[q];}};
  const events=[];
  setup({parentElement:root,data,setStateValue:(key,obj)=>{if(key==='event')events.push(obj);}});
  function evt(canvas,name,x,y){
    const handler=canvas.handlers.get(name);assert.ok(handler,`Missing ${name}`);
    handler({clientX:x,clientY:y,pointerId:10,preventDefault:noop});
  }
  const size=396,left=120,top=28,span=data.workspace_limit;
  const pixel=(x,y)=>({x:left+(x+span)/(2*span)*size,y:top+(span-y)/(2*span)*size});
  const drag=(canvas,start,stop)=>{
    evt(canvas,'pointerdown',start.x,start.y);
    evt(canvas,'pointermove',stop.x,stop.y);
    evt(canvas,'pointerup',stop.x,stop.y);
    return events.at(-1);
  };
  return {w,c,events,pixel,drag,evt,root};
}
const r=(a,theta=0)=>({type:'R',a,theta,s:0,stroke:3});
const p=(a,s,stroke,theta=0)=>({type:'P',a,theta,s,stroke});
const defaultData=(joints,limit,axes=[0,1])=>({joints,workspace_limit:limit,
    axis_x:axes[0],axis_y:axes[1],obstacles:[[1.1,1.1],[1.1,-1.1]],
    quality:2,show_angle_arcs:true,show_labels:true,
    link_half_width:.125,obstacle_radius:.125});

// Two revolute joints with unequal lengths: elbow and TCP are draggable.
{
  const t=run(defaultData([r(1.5),r(.5)],2.35));
  const elbow=t.drag(t.w,t.pixel(1.5,0),t.pixel(0,1.5));
  assert.equal(elbow.kind,'joint');close(elbow.values[0],Math.PI/2);
  const target=[1.25,1.0];
  const tcp=t.drag(t.w,t.pixel(0,2),t.pixel(...target));
  assert.equal(tcp.kind,'tool');
  const x=1.5*Math.cos(tcp.values[0])+.5*Math.cos(tcp.values[0]+tcp.values[1]);
  const y=1.5*Math.sin(tcp.values[0])+.5*Math.sin(tcp.values[0]+tcp.values[1]);
  close(x,target[0],1e-8);close(y,target[1],1e-8);
  const ob=t.drag(t.w,t.pixel(1.1,-1.1),t.pixel(-1.9,-1.75));
  assert.equal(ob.kind,'obstacle');close(ob.obstacles[1][0],-1.9);
  const cx=120+396*.25,cy=28+396*.75;
  t.evt(t.c,'pointerdown',cx,cy);t.evt(t.c,'pointermove',cx,cy);t.evt(t.c,'pointerup',cx,cy);
  const ce=t.events.at(-1);assert.equal(ce.kind,'configuration');
  close(ce.values[0],Math.PI/2);close(ce.values[1],Math.PI/2);
  t.root.__rmCleanup();
}
// R/P planar extension: drag the intermediate P slider, >5 link length,
// obstacle can move outside old [-2,2] square, and P C-space slider works.
{
  const t=run(defaultData([r(7),p(2,2,8),r(1)],18,[0,1]));
  // P endpoint is at x=11; the tool is x=12.
  const slide=t.drag(t.w,t.pixel(11,0),t.pixel(15,0));
  assert.equal(slide.kind,'joint');close(slide.values[1],6);
  const moved=t.drag(t.w,t.pixel(1.1,1.1),t.pixel(9.2,5));
  close(moved.obstacles[0][0],9.2);close(moved.obstacles[0][1],5);
  const x=120+396*.2, y=28+396*.25; // P s=6 and R theta=72deg
  t.evt(t.c,'pointerdown',x,y);t.evt(t.c,'pointerup',x,y);
  close(t.events.at(-1).values[1],6);
  close(t.events.at(-1).values[0],2*Math.PI*.2);
  t.root.__rmCleanup();
}
// Three R, test numerical solver near a reachable point.
{
  const t=run(defaultData([r(1),r(1),r(1)],3.35));
  const result=t.drag(t.w,t.pixel(3,0),t.pixel(1.8,.8));
  assert.equal(result.kind,'tool');
  let x=0,y=0,angle=0;
  for(const v of result.values){angle+=v;x+=Math.cos(angle);y+=Math.sin(angle);}
  assert.ok(Math.hypot(x-1.8,y-.8)<.18,`Numeric IK too far: ${x},${y}`);
  t.root.__rmCleanup();
}
console.log('PASS: 2R IK, dynamic joint drags, R/P slider, large coordinates, C-space axes, multi-R numeric IK');
