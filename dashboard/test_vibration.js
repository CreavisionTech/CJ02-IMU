/* Node smoke test for the browser-only vibration workflow. */
const fs=require("fs"),vm=require("vm"),assert=require("assert");
class ClassList{toggle(){} }
class El{
  constructor(id=""){this.id=id;this.disabled=false;this.textContent="";this.innerHTML="";this.className="";this.classList=new ClassList();this.dataset={};this.parentNode={insertBefore(){}}}
  getContext(){return new Proxy({setTransform(){},clearRect(){},fillRect(){},beginPath(){},moveTo(){},lineTo(){},stroke(){},fillText(){},setLineDash(){}},{set(o,k,v){o[k]=v;return true}})}
  getBoundingClientRect(){return{width:700,height:290}}
  insertAdjacentHTML(_where,html){this.innerHTML+=html}
}
const ids=["vibExport","vibBaseCount","vibRunCount","vibValCount","vibBase","vibRun","vibStop","vibAnalyze","vibApply","vibSave","vibValidate","vibFilterReset","vibReset","vibStatus","vibChart","vibResults","vibOutputRate","vibBaud"];
const elements=Object.fromEntries(ids.map(id=>[id,new El(id)]));
const steps=["baseline","run","validation"].map(k=>{const e=new El();e.dataset.step=k;return e});
const document={
  querySelector(sel){return sel===".hint"?new El():null},
  querySelectorAll(sel){return sel===".vib-step"?steps:[]},
  createElement(tag){return new El(tag)},getElementById(id){return elements[id]}
};
const context={document,window:{},state:{connected:true},devicePixelRatio:1,console,setTimeout,clearTimeout,Math,Date,Float64Array,Uint16Array,Blob:function(){},URL:{createObjectURL(){return"blob:test"},revokeObjectURL(){}}};
context.window=context;vm.createContext(context);
vm.runInContext(fs.readFileSync(__dirname+"/vibration.js","utf8"),context);
function sample(i,active){
  const n=(i*1103515245+12345)&0x7fffffff,noise=((n%101)-50)/100;
  const a=active?1.8*Math.sin(2*Math.PI*120*i/1600):.01*noise;
  const g=active?120*Math.sin(2*Math.PI*180*i/1600):.1*noise;
  return{acc:[Math.round(a*2048),0,2048],gyr:[Math.round(g*16.4),0,0],seq:i&65535};
}
elements.vibBase.onclick();for(let i=0;i<6400;i++)context.vibrationAddSample(sample(i,false));elements.vibStop.onclick();
elements.vibRun.onclick();for(let i=0;i<6400;i++)context.vibrationAddSample(sample(i,true));elements.vibStop.onclick();
assert.strictEqual(elements.vibAnalyze.disabled,false);elements.vibAnalyze.onclick();
setTimeout(()=>{
  assert.match(elements.vibResults.innerHTML,/value="120\.3"/);
  assert.match(elements.vibResults.innerHTML,/value="179\.7"/);
  context.vibrationHandleReply(0x05,true,new Uint8Array());
  assert.strictEqual(elements.vibValidate.disabled,false);
  elements.vibValidate.onclick();for(let i=0;i<4800;i++)context.vibrationAddSample(sample(i,true));elements.vibStop.onclick();
  assert.match(elements.vibResults.innerHTML,/上机复测/);
  console.log("PASS: baseline, run, 1 s FFT, 120/180 Hz peaks, replay, validation");
},100);
