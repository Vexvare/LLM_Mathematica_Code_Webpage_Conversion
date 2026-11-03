"""High-performance, two-link planar DH robot interaction component.

The workspace and configuration space are rendered together in one Streamlit
custom component. During a drag, both views update entirely in the browser.
Python receives the final state only when the interaction is committed.

This keeps the interaction responsive while preserving the Python model as the
canonical implementation used by the rest of the application.
"""

from __future__ import annotations

from typing import Iterable

import numpy as np
import streamlit as st



_COMPONENT_HTML = """
<div class="rm-component">
    <div class="rm-grid">
        <section class="rm-panel">
            <div class="rm-panel-title">Robot Workspace</div>
            <canvas class="rm-workspace"></canvas>
            <div class="rm-panel-hint">
                Drag obstacles, intermediate joints, or the tool.
            </div>
        </section>

        <section class="rm-panel">
            <div class="rm-panel-title">Configuration Space</div>
            <canvas class="rm-cspace"></canvas>
            <div class="rm-panel-hint">
                Click or drag the red crosshair (selected two joints).
            </div>
        </section>
    </div>
</div>
"""


_COMPONENT_CSS = """
.rm-component {
    width: 100%;
    min-width: 0;
    box-sizing: border-box;
}

.rm-grid {
    display: grid;
    grid-template-columns: minmax(0, 1fr) minmax(0, 1fr);
    gap: 1rem;
    width: 100%;
}

.rm-panel {
    min-width: 0;
    border: 1px solid rgba(128, 128, 128, 0.20);
    border-radius: 0.75rem;
    padding: 0.75rem;
    box-sizing: border-box;
    background: var(--st-background-color, #ffffff);
}

.rm-panel-title {
    text-align: center;
    font-family: sans-serif;
    font-weight: 700;
    font-size: 0.98rem;
    margin: 0.05rem 0 0.4rem 0;
}

.rm-panel-hint {
    text-align: center;
    font-family: sans-serif;
    font-size: 0.76rem;
    opacity: 0.68;
    margin-top: 0.35rem;
    line-height: 1.3;
}

.rm-workspace,
.rm-cspace {
    display: block;
    width: 100%;
    height: 470px;
    min-width: 0;
    border-radius: 0.45rem;
    background: #ffffff;
    touch-action: none;
    user-select: none;
}

.rm-workspace {
    cursor: default;
}

.rm-cspace {
    cursor: crosshair;
}
"""


_COMPONENT_JS = r"""
export default function(component) {
  const {parentElement, data, setStateValue} = component;
  const grid = parentElement.querySelector('.rm-grid');
  const workspaceCanvas = parentElement.querySelector('.rm-workspace');
  const cspaceCanvas = parentElement.querySelector('.rm-cspace');
  if (!grid || !workspaceCanvas || !cspaceCanvas) throw Error('Missing robot canvases');
  if (parentElement.__rmCleanup) parentElement.__rmCleanup();
  const TAU = Math.PI * 2;
  const clamp = (x,a,b)=>Math.max(a,Math.min(b,x));
  const wrap = x=>((x%TAU)+TAU)%TAU;
  const MAX_VIEW = Number(data.workspace_limit ?? 2.2);
  const halfWidth = Number(data.link_half_width ?? 0.125);
  // Filled, user-sized obstacles. Link geometry is a capsule of radius halfWidth.
  function buildVertices(o) {
    if (o.shape==='Circle') return [];
    let w=o.width,h=o.shape==='Square'?o.width:o.height;
    const local=o.shape==='Triangle'?[[-w/2,-h/3],[w/2,-h/3],[0,2*h/3]]:
      [[-w/2,-h/2],[w/2,-h/2],[w/2,h/2],[-w/2,h/2]];
    const t=o.rotation_deg*Math.PI/180,c=Math.cos(t),s=Math.sin(t);
    return local.map(([x,y])=>[o.x+c*x-s*y,o.y+s*x+c*y]);
  }
  function toObstacle(p) {
    const o=Array.isArray(p)?{x:Number(p[0]),y:Number(p[1]),shape:'Circle'}:{...p};
    o.x=Number(o.x);o.y=Number(o.y);o.shape=o.shape||'Circle';
    o.radius=Number(o.radius??.125);o.width=Number(o.width??.5);
    o.height=Number(o.height??.5);o.rotation_deg=Number(o.rotation_deg??0);
    o.vertices=buildVertices(o);return o;
  }
  const state = {
    joints: data.joints.map(j=>({type:j.type, a:Number(j.a), theta:Number(j.theta),
                                 s:Number(j.s), stroke:Number(j.stroke)})),
    obstacles: data.obstacles.map(toObstacle),
    axisX: Number(data.axis_x), axisY: Number(data.axis_y),
    quality: clamp(Number(data.quality),1,10),
    labels: Boolean(data.show_labels), arcs:Boolean(data.show_angle_arcs),
    raster:null, res:0, isPreview:false, wGeom:null, cGeom:null,
    drag:null, cDragging:false, frame:null, destroyed:false,
  };
  const colors={free:[248,250,252,255],blocked:[51,65,85,255],red:'#d62728',
    arm:'#59636e',blue:'#2563eb',purple:'#7c3aed',orange:'#c2410c',
    text:'#334155',grid:'#e5e7eb',axis:'#64748b'};
  function variables(){return state.joints.map(j=>j.type==='R'?j.theta:j.s);}
  function varMax(i){return state.joints[i].type==='R'?TAU:state.joints[i].stroke;}
  function value(i){
    const j=state.joints[i];
    return j.type==='R'?(Math.abs(j.theta-TAU)<1e-10?TAU:wrap(j.theta)):j.s;
  }
  function setValue(i,v){
    const j=state.joints[i];
    if(j.type==='R')j.theta=Math.abs(v-TAU)<1e-10?TAU:wrap(v);
    else j.s=clamp(v,0,j.stroke);
  }
  function getPose(){
    let x=0,y=0,heading=0;
    const points=[[0,0]], headings=[];
    for (const j of state.joints){
      heading+=j.theta;
      headings.push(heading);
      const L=j.a+(j.type==='P'?j.s:0);
      x+=L*Math.cos(heading);y+=L*Math.sin(heading);
      points.push([x,y]);
    }
    return {points,headings,end:points.at(-1)};
  }
  function segmentDistanceSquared(px,py,ax,ay,bx,by){
    const vx=bx-ax,vy=by-ay,denom=vx*vx+vy*vy;
    const t=denom<1e-14?0:clamp(((px-ax)*vx+(py-ay)*vy)/denom,0,1);
    const dx=px-(ax+t*vx),dy=py-(ay+t*vy);return dx*dx+dy*dy;
  }
  function orientation(ax,ay,bx,by,px,py) {
    return (bx-ax)*(py-ay)-(by-ay)*(px-ax);
  }
  function segmentsIntersect(a,b,c,d) {
    const u=orientation(...a,...b,...c),v=orientation(...a,...b,...d);
    const w=orientation(...c,...d,...a),z=orientation(...c,...d,...b);
    const eps=1e-12;
    const opposites=(x,y)=>(x>=-eps&&y<=eps)||(x<=eps&&y>=-eps);
    return opposites(u,v)&&opposites(w,z)&&
      Math.max(Math.min(a[0],b[0]),Math.min(c[0],d[0]))<=
      Math.min(Math.max(a[0],b[0]),Math.max(c[0],d[0]))+eps &&
      Math.max(Math.min(a[1],b[1]),Math.min(c[1],d[1]))<=
      Math.min(Math.max(a[1],b[1]),Math.max(c[1],d[1]))+eps;
  }
  function inPolygon(p,poly) {
    for (let k=0;k<poly.length;k++) {
      if(orientation(...poly[k],...poly[(k+1)%poly.length],...p)<-1e-12)return false;
    }
    return true;
  }
  function linkHitsObstacle(a,b,o){
    if(o.shape==='Circle'){
      return segmentDistanceSquared(o.x,o.y,...a,...b)<(halfWidth+o.radius)**2;
    }
    const vs=o.vertices;
    if(inPolygon(a,vs)||inPolygon(b,vs))return true;
    const tol2=halfWidth*halfWidth;
    for(let k=0;k<vs.length;k++){
      const c=vs[k],d=vs[(k+1)%vs.length];
      if(segmentsIntersect(a,b,c,d))return true;
      if(segmentDistanceSquared(...a,...c,...d)<tol2 ||
         segmentDistanceSquared(...b,...c,...d)<tol2 ||
         segmentDistanceSquared(...c,...a,...b)<tol2)return true;
    }
    return false;
  }
  function hitLink(a,b){return state.obstacles.some(o=>linkHitsObstacle(a,b,o));}
  function inCollision(pose){for(let i=0;i<state.joints.length;i++)
      if(hitLink(pose.points[i],pose.points[i+1]))return true;
      return false;
  }
  function info(canvas){
    const b=canvas.getBoundingClientRect(),dpr=window.devicePixelRatio||1;
    const w=Math.max(1,b.width),h=Math.max(1,b.height);
    if(canvas.width!==Math.round(w*dpr)||canvas.height!==Math.round(h*dpr)){
      canvas.width=Math.round(w*dpr);canvas.height=Math.round(h*dpr);
    }
    const ctx=canvas.getContext('2d');ctx.setTransform(dpr,0,0,dpr,0,0);
    const leftMargin=54,rightMargin=18,topMargin=28,bottomMargin=46;
    const size=Math.max(35,Math.min(w-leftMargin-rightMargin,h-topMargin-bottomMargin));
    const left=leftMargin+Math.max(0,(w-leftMargin-rightMargin-size)/2);
    const top=topMargin+Math.max(0,(h-topMargin-bottomMargin-size)/2);
    return {ctx,w,h,left,top,size,right:left+size,bottom:top+size};
  }
  function workspacePixel(x,y,g){return {x:g.left+(x+MAX_VIEW)/(2*MAX_VIEW)*g.size,
      y:g.top+(MAX_VIEW-y)/(2*MAX_VIEW)*g.size};}
  function workspacePoint(x,y,g){return {x:(x-g.left)/g.size*2*MAX_VIEW-MAX_VIEW,
      y:MAX_VIEW-(y-g.top)/g.size*2*MAX_VIEW};}
  function cspacePixel(g){return{x:g.left+value(state.axisX)/varMax(state.axisX)*g.size,
       y:g.bottom-value(state.axisY)/varMax(state.axisY)*g.size};}
  function cspacePoint(px,py,g){return{x:clamp((px-g.left)/g.size,0,1)*varMax(state.axisX),
       y:clamp((g.bottom-py)/g.size,0,1)*varMax(state.axisY)};}
  function drawAxes(g,xticks,yticks,xLabel,yLabel){
    const c=g.ctx;
    c.save();c.strokeStyle=colors.grid;c.lineWidth=1;
    c.font='11px sans-serif';c.fillStyle=colors.text;c.textAlign='center';c.textBaseline='top';
    for(const t of xticks){c.beginPath();c.moveTo(t.pixel,g.top);c.lineTo(t.pixel,g.bottom);c.stroke();c.fillText(t.label,t.pixel,g.bottom+7);}
    c.textAlign='right';c.textBaseline='middle';
    for(const t of yticks){c.beginPath();c.moveTo(g.left,t.pixel);c.lineTo(g.right,t.pixel);c.stroke();c.fillText(t.label,g.left-8,t.pixel);}
    c.strokeStyle=colors.axis;c.strokeRect(g.left,g.top,g.size,g.size);
    c.fillStyle=colors.text;c.font='12px sans-serif';c.textBaseline='bottom';c.textAlign='center';
    c.fillText(xLabel,g.left+g.size/2,g.bottom+42);
    c.save();c.translate(g.left-42,g.top+g.size/2);c.rotate(-Math.PI/2);c.fillText(yLabel,0,0);c.restore();c.restore();
  }
  function workspaceTicks(){
    const raw=2*MAX_VIEW/8;
    const exp=Math.floor(Math.log10(raw));const base=10**exp;
    const step=[1,2,5,10].map(x=>x*base).find(x=>x>=raw)||10*base;
    const values=[];const m=Math.floor(MAX_VIEW/step);
    for(let k=-m;k<=m;k++)values.push(k*step);
    return values;
  }
  function drawWorkspace(){
    if(state.destroyed)return;
    const g=info(workspaceCanvas);state.wGeom=g;const c=g.ctx;
    c.clearRect(0,0,g.w,g.h);c.fillStyle='white';c.fillRect(0,0,g.w,g.h);
    const ticks=workspaceTicks();
    drawAxes(g,ticks.map(n=>({label:String(+n.toPrecision(5)),pixel:workspacePixel(n,0,g).x})),
      ticks.map(n=>({label:String(+n.toPrecision(5)),pixel:workspacePixel(0,n,g).y})),'x','y');
    const scale=g.size/(2*MAX_VIEW),pose=getPose();
    for(let i=0;i<state.obstacles.length;i++){
      const o=state.obstacles[i],p=workspacePixel(o.x,o.y,g);
      c.beginPath();
      if(o.shape==='Circle')c.arc(p.x,p.y,Math.max(.1,o.radius*scale),0,TAU);
      else{
        o.vertices.forEach((v,k)=>{
          const q=workspacePixel(...v,g);if(k===0)c.moveTo(q.x,q.y);else c.lineTo(q.x,q.y);
        });c.closePath();
      }
      c.fillStyle='rgba(249,115,22,.45)';c.fill();c.lineWidth=2.5;c.strokeStyle=colors.orange;c.stroke();
      // The center dot is draggable even if the obstacle is tiny at current zoom.
      c.beginPath();c.arc(p.x,p.y,4,0,TAU);c.fillStyle=colors.orange;c.fill();
      if(state.labels){c.fillStyle=colors.orange;c.font='600 12px sans-serif';c.textAlign='center';
        c.fillText('O'+(i+1)+' · '+o.shape,p.x,p.y-10);}
    }
    for(let i=0;i<state.joints.length;i++){
      const a=workspacePixel(...pose.points[i],g),b=workspacePixel(...pose.points[i+1],g);
      const isHit=hitLink(pose.points[i],pose.points[i+1]);
      c.strokeStyle=isHit?colors.red:colors.arm;c.lineWidth=Math.max(1,2*halfWidth*scale);c.lineCap='round';
      c.beginPath();c.moveTo(a.x,a.y);c.lineTo(b.x,b.y);c.stroke();
      if(state.arcs && state.joints[i].type==='R'){
        const start=pose.headings[i]-state.joints[i].theta;
        c.beginPath();c.setLineDash([4,3]);c.lineWidth=1.5;c.strokeStyle=colors.axis;
        c.arc(a.x,a.y,Math.max(14,Math.min(36,.3*scale)),-start,-pose.headings[i],state.joints[i].theta>Math.PI);
        c.stroke();c.setLineDash([]);
      }
    }
    const base=workspacePixel(...pose.points[0],g);c.fillStyle='#111827';
    c.beginPath();c.arc(base.x,base.y,6,0,TAU);c.fill();
    for(let i=1;i<pose.points.length;i++){
      const p=workspacePixel(...pose.points[i],g);const tip=i===pose.points.length-1;
      c.beginPath();c.arc(p.x,p.y,tip?11:9,0,TAU);
      c.fillStyle=tip?'rgba(124,58,237,.18)':'rgba(37,99,235,.16)';c.fill();
      c.strokeStyle=tip?colors.purple:colors.blue;c.lineWidth=3;c.stroke();
      if(state.labels){c.font='600 11px sans-serif';c.textAlign='left';c.fillStyle=tip?colors.purple:colors.blue;
        c.fillText(tip?'TCP':`J${i}`,p.x+10,p.y-10);}
    }
    c.font='700 12px sans-serif';c.fillStyle=inCollision(pose)?colors.red:'#166534';c.textAlign='left';
    c.fillText(inCollision(pose)?'COLLISION':'Collision-free',g.left+8,g.top+18);
  }
  function axisTicks(index,g,horizontal){
    const hi=varMax(index),ticks=[];
    for(let i=0;i<=4;i++){
      const v=i*hi/4,p=horizontal?g.left+i*g.size/4:g.bottom-i*g.size/4;
      ticks.push({pixel:p,label:state.joints[index].type==='R'?String(Math.round(v*180/Math.PI))+'°':Number(v.toPrecision(3)).toString()});
    }
    return ticks;
  }
  function drawCspace(){
    if(state.destroyed)return;
    const g=info(cspaceCanvas);state.cGeom=g;const c=g.ctx;
    c.clearRect(0,0,g.w,g.h);c.fillStyle='white';c.fillRect(0,0,g.w,g.h);
    const name=index=>`J${index+1} ${state.joints[index].type==='R'?'θ (°)':'s (units)'}`;
    drawAxes(g,axisTicks(state.axisX,g,true),axisTicks(state.axisY,g,false),name(state.axisX),name(state.axisY));
    if(state.raster){c.imageSmoothingEnabled=false;c.drawImage(state.raster,g.left,g.top,g.size,g.size);}
    const p=cspacePixel(g);c.strokeStyle='#dc2626';c.lineWidth=2.5;c.beginPath();
    c.moveTo(p.x-11,p.y);c.lineTo(p.x+11,p.y);c.moveTo(p.x,p.y-11);c.lineTo(p.x,p.y+11);c.stroke();
    c.beginPath();c.arc(p.x,p.y,4,0,TAU);c.fillStyle='#dc2626';c.fill();
    c.fillStyle=colors.text;c.font='11px sans-serif';c.textAlign='right';
    c.fillText(`${state.res} × ${state.res}${state.isPreview?' preview':''}`,g.right-5,g.top+15);
    if(state.joints.length>2){c.textAlign='center';c.font='10px sans-serif';
      c.fillText('Other joints fixed at current values',g.left+g.size/2,g.top+28);}
  }
  function computeCspace(resolution,preview){
    const off=document.createElement('canvas');off.width=resolution;off.height=resolution;
    const context=off.getContext('2d'),img=context.createImageData(resolution,resolution);
    const xs=Array.from({length:resolution},(_,i)=>(i+.5)/resolution*varMax(state.axisX));
    const ys=Array.from({length:resolution},(_,i)=>(i+.5)/resolution*varMax(state.axisY));
    for(let j=0;j<resolution;j++)for(let i=0;i<resolution;i++){
      let x=0,y=0,heading=0,collide=false;
      for(let k=0;k<state.joints.length;k++){
        const joint=state.joints[k];
        const variable=k===state.axisX?xs[i]:k===state.axisY?ys[j]:value(k);
        const theta=joint.type==='R'?variable:joint.theta;
        const s=joint.type==='P'?variable:0;
        heading+=theta;
        const nx=x+(joint.a+s)*Math.cos(heading),ny=y+(joint.a+s)*Math.sin(heading);
        for(const o of state.obstacles){
          if(linkHitsObstacle([x,y],[nx,ny],o)){collide=true;break;}
        }
        x=nx;y=ny;if(collide)break;
      }
      const row=resolution-1-j,offset=4*(row*resolution+i),rgba=collide?colors.blocked:colors.free;
      img.data[offset]=rgba[0];img.data[offset+1]=rgba[1];img.data[offset+2]=rgba[2];img.data[offset+3]=255;
    }
    context.putImageData(img,0,0);state.raster=off;state.res=resolution;state.isPreview=preview;
  }
  function recalcPreview(){
    if(state.frame!==null)return;
    state.frame=requestAnimationFrame(()=>{state.frame=null;
      if(state.destroyed)return;computeCspace(8*state.quality,true);drawCspace();});
  }
  function finishMap(){
    if(state.frame!==null){cancelAnimationFrame(state.frame);state.frame=null;}
    computeCspace(32*state.quality,false);drawCspace();
  }
  function emit(kind){setStateValue('event',{
    kind,values:variables().map(Number),obstacles:state.obstacles.map(o=>[Number(o.x),Number(o.y)]),
    nonce:Date.now()+Math.random(),
  });}
  function position(e,canvas){const r=canvas.getBoundingClientRect();return{x:e.clientX-r.left,y:e.clientY-r.top};}
  function dist(a,b){return Math.hypot(a.x-b.x,a.y-b.y);}
  function hitTest(pos){
    const g=state.wGeom;if(!g)return null;
    const pose=getPose();let best=null,bestDistance=Infinity;
    const candidate=(kind,index,p,radius)=>{
      const d=dist(pos,p);
      if(d<=radius&&d<bestDistance){best={kind,index};bestDistance=d;}
    };
    candidate('tool',null,workspacePixel(...pose.end,g),16);
    for(let i=pose.points.length-2;i>=1;i--){
      candidate('joint',i-1,workspacePixel(...pose.points[i],g),16);
    }
    for(let i=0;i<state.obstacles.length;i++){
      const o=state.obstacles[i];
      const radius=o.shape==='Circle'?o.radius:Math.hypot(o.width,o.shape==='Square'?o.width:o.height)/2;
      const hitRadius=Math.max(14,radius*g.size/(2*MAX_VIEW)+5);
      candidate('obstacle',i,workspacePixel(o.x,o.y,g),hitRadius);
    }
    return best;
  }
  function solve2R(targetX,targetY){
    let l1=state.joints[0].a,l2=state.joints[1].a;
    let r=Math.hypot(targetX,targetY),lo=Math.abs(l1-l2),hi=l1+l2;
    if(r>hi&&r>0){const k=hi/r;targetX*=k;targetY*=k;r=hi;}
    else if(r<lo){if(r>0){const k=lo/r;targetX*=k;targetY*=k;}else{targetX=lo;targetY=0;}r=lo;}
    const c=clamp((targetX*targetX+targetY*targetY-l1*l1-l2*l2)/(2*l1*l2),-1,1);
    const current=variables();let best=null;
    for(const t2 of [Math.acos(c),-Math.acos(c)]){
      const t1=wrap(Math.atan2(targetY,targetX)-Math.atan2(l2*Math.sin(t2),l1+l2*Math.cos(t2)));
      const a2=wrap(t2),delta=(x,y)=>Math.abs(((x-y+Math.PI+TAU)%TAU)-Math.PI);
      const cost=delta(t1,current[0])+delta(a2,current[1]);
      if(!best||cost<best.cost)best={cost,t1,t2:a2};
    }
    setValue(0,best.t1);setValue(1,best.t2);
  }
  function solveNumeric(targetX,targetY){
    // Damped least squares, 2D Cartesian error, mixed R/P Jacobian.
    const joints=state.joints;
    const base=variables();let best=base.slice(),bestError=Infinity;
    for(let restart=0;restart<3;restart++){
      if(restart){for(let k=0;k<joints.length;k++) if(joints[k].type==='R')
        setValue(k,base[k]+(restart===1?.14:-.14)*(k%2===0?1:-1));}
      for(let iter=0;iter<90;iter++){
        const pose=getPose(),ex=targetX-pose.end[0],ey=targetY-pose.end[1];
        const error=Math.hypot(ex,ey);
        if(error<bestError){bestError=error;best=variables();}
        if(error<1e-5)break;
        const jac=jointJacobian(pose);
        let aa=.0025,bb=0,cc=.0025;
        for(const col of jac){aa+=col[0]*col[0];bb+=col[0]*col[1];cc+=col[1]*col[1];}
        const det=aa*cc-bb*bb;if(det<1e-15)break;
        const u=(cc*ex-bb*ey)/det,v=(-bb*ex+aa*ey)/det;
        const steps=jac.map(col=>col[0]*u+col[1]*v);
        let accepted=false;
        for(const factor of [1,.5,.25,.1]){
          const prev=variables();
          for(let k=0;k<joints.length;k++){
            const maxStep=joints[k].type==='R'?.26:Math.max(.05,joints[k].stroke/12);
            setValue(k,prev[k]+clamp(steps[k],-maxStep,maxStep)*factor);
          }
          const next=getPose().end;
          if(Math.hypot(targetX-next[0],targetY-next[1])<error-.00000001){accepted=true;break;}
          for(let k=0;k<joints.length;k++)setValue(k,prev[k]);
        }
        if(!accepted)break;
      }
    }
    for(let k=0;k<joints.length;k++)setValue(k,best[k]);
  }
  function jointJacobian(pose){return state.joints.map((j,i)=>{
    if(j.type==='R')return[-(pose.end[1]-pose.points[i][1]),pose.end[0]-pose.points[i][0]];
    return[Math.cos(pose.headings[i]),Math.sin(pose.headings[i])];
  });}
  function moveJoint(index,x,y){
    const pose=getPose(),origin=pose.points[index],joint=state.joints[index];
    if(joint.type==='R'){
      let previousHeading=index?pose.headings[index-1]:0;
      setValue(index,Math.atan2(y-origin[1],x-origin[0])-previousHeading);
    }else{
      const direction=pose.headings[index];
      setValue(index,(x-origin[0])*Math.cos(direction)+(y-origin[1])*Math.sin(direction)-joint.a);
    }
  }
  function workspaceDown(e){
    const h=hitTest(position(e,workspaceCanvas));if(!h)return;
    e.preventDefault();state.drag={...h,id:e.pointerId};workspaceCanvas.setPointerCapture?.(e.pointerId);
  }
  function workspaceMove(e){
    const drag=state.drag;if(!drag||drag.id!==e.pointerId||!state.wGeom)return;
    e.preventDefault();const pos=position(e,workspaceCanvas),p=workspacePoint(pos.x,pos.y,state.wGeom);
    if(drag.kind==='obstacle'){
      const o=state.obstacles[drag.index];
      o.x=clamp(p.x,-MAX_VIEW,MAX_VIEW);o.y=clamp(p.y,-MAX_VIEW,MAX_VIEW);
      o.vertices=buildVertices(o);
      drawWorkspace();recalcPreview();
    }else if(drag.kind==='joint'){
      moveJoint(drag.index,p.x,p.y);drawWorkspace();drawCspace();
      if(drag.index!==state.axisX&&drag.index!==state.axisY)recalcPreview();
    }else if(drag.kind==='tool'){
      if(state.joints.length===2&&state.joints.every(j=>j.type==='R'))solve2R(p.x,p.y);
      else solveNumeric(p.x,p.y);
      drawWorkspace();drawCspace();
      if(state.joints.some((_,i)=>i!==state.axisX&&i!==state.axisY))recalcPreview();
    }
  }
  function workspaceUp(e){
    if(!state.drag||state.drag.id!==e.pointerId)return;
    e.preventDefault();const kind=state.drag.kind;state.drag=null;finishMap();emit(kind);
  }
  function cspaceUpdate(e){
    const g=state.cGeom;if(!g)return;
    const p=position(e,cspaceCanvas),v=cspacePoint(p.x,p.y,g);
    setValue(state.axisX,v.x);setValue(state.axisY,v.y);
    drawWorkspace();drawCspace();
  }
  function cspaceDown(e){
    const g=state.cGeom;if(!g)return;const p=position(e,cspaceCanvas);
    if(p.x<g.left||p.x>g.right||p.y<g.top||p.y>g.bottom)return;
    e.preventDefault();state.cDragging=true;cspaceCanvas.setPointerCapture?.(e.pointerId);cspaceUpdate(e);
  }
  function cspaceMove(e){if(!state.cDragging)return;e.preventDefault();cspaceUpdate(e);}
  function cspaceUp(e){if(!state.cDragging)return;e.preventDefault();cspaceUpdate(e);state.cDragging=false;emit('configuration');}
  workspaceCanvas.addEventListener('pointerdown',workspaceDown);
  workspaceCanvas.addEventListener('pointermove',workspaceMove);
  workspaceCanvas.addEventListener('pointerup',workspaceUp);
  workspaceCanvas.addEventListener('pointercancel',workspaceUp);
  cspaceCanvas.addEventListener('pointerdown',cspaceDown);
  cspaceCanvas.addEventListener('pointermove',cspaceMove);
  cspaceCanvas.addEventListener('pointerup',cspaceUp);
  cspaceCanvas.addEventListener('pointercancel',cspaceUp);
  let prevW=workspaceCanvas.getBoundingClientRect().width,prevC=cspaceCanvas.getBoundingClientRect().width;
  const ro=typeof ResizeObserver!=='undefined'?new ResizeObserver(()=>{
    const w=workspaceCanvas.getBoundingClientRect().width,c=cspaceCanvas.getBoundingClientRect().width;
    if(Math.abs(w-prevW)<1&&Math.abs(c-prevC)<1)return;
    prevW=w;prevC=c;requestAnimationFrame(()=>{drawWorkspace();drawCspace();});
  }):null;
  ro?.observe(grid);
  computeCspace(32*state.quality,false);drawWorkspace();drawCspace();
  const cleanup=()=>{
    state.destroyed=true;if(state.frame!==null)cancelAnimationFrame(state.frame);
    ro?.disconnect();
    for(const [cv,handlers] of [[workspaceCanvas,[workspaceDown,workspaceMove,workspaceUp,workspaceUp]],
                                [cspaceCanvas,[cspaceDown,cspaceMove,cspaceUp,cspaceUp]]]){
      ['pointerdown','pointermove','pointerup','pointercancel'].forEach((name,i)=>cv.removeEventListener(name,handlers[i]));
    }
  };
  parentElement.__rmCleanup=cleanup;
  return cleanup;
}

"""


_interactive_robot = st.components.v2.component(
    "robot_motion_dh_multilink_interaction",
    html=_COMPONENT_HTML, css=_COMPONENT_CSS, js=_COMPONENT_JS,
)


def render_interactive_dh_system(*, joints, obstacles, axis_x, axis_y,
                                 workspace_limit, quality, show_angle_arcs,
                                 show_labels, key):
    import numpy as np
    joints = tuple(joints)
    if len(joints) < 2 or len(joints) > 12:
        raise ValueError("Expected 2 to 12 joints")
    from ui.obstacle_geometry import parse_obstacles
    obs = parse_obstacles(obstacles)
    if len(obs) != 2:
        raise ValueError("Expected exactly two 2D obstacles")
    if not 0 <= axis_x < len(joints) or not 0 <= axis_y < len(joints) or axis_x == axis_y:
        raise ValueError("Choose distinct C-space joint axes")
    return _interactive_robot(
        data={
            "joints": [
                {"type": j.kind, "a": float(j.a),
                 "theta": float(np.deg2rad(j.theta_deg)),
                 "s": float(j.extension), "stroke": float(j.stroke)}
                for j in joints
            ],
            "obstacles": [o.to_dict() for o in obs], "axis_x": int(axis_x), "axis_y": int(axis_y),
            "workspace_limit": float(workspace_limit), "quality": int(quality),
            "show_angle_arcs": bool(show_angle_arcs),
            "show_labels": bool(show_labels),
            "link_half_width": .125, "obstacle_radius": .125,
        },
        default={"event": None}, key=key, on_event_change=lambda: None,
        width="stretch", height=550,
    )
