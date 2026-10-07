// صفحة الويب العربية لوحدة الشاشة (ملف منفصل لأن معالج Arduino يتعثر بالنصوص الخام)
#pragma once
#include <pgmspace.h>
const char PAGE[] PROGMEM = R"HTML(<!DOCTYPE html><html lang="ar" dir="rtl"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>TPMS V3</title><style>
:root{--bg:#0b0d12;--card:#141a26;--bd:#243049;--tx:#e8edf5;--mu:#8a96ab;--ac:#22c5ee;--ok:#22c55e;--wa:#f59e0b;--da:#ef4444}
*{box-sizing:border-box}body{margin:0;font-family:system-ui,sans-serif;background:var(--bg);color:var(--tx)}
header{padding:12px 16px;background:#0f141e;border-bottom:1px solid var(--bd);display:flex;justify-content:space-between;align-items:center;flex-wrap:wrap;gap:6px}
.tabs{display:flex;gap:6px;padding:10px;flex-wrap:wrap}.tb{background:var(--card);color:var(--tx);border:1px solid var(--bd);padding:8px 14px;border-radius:8px;cursor:pointer}
.tb.on{background:var(--ac);color:#000;font-weight:700}.pg{display:none;padding:0 10px 20px}.pg.on{display:block}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:10px;direction:ltr}.grid>*{direction:rtl}
.c{background:var(--card);border:2px solid var(--bd);border-radius:12px;padding:12px;margin-bottom:10px}
.c.ok{border-color:#22c55e55}.c.wa{border-color:var(--wa)}.c.da{border-color:var(--da);background:#2a0c0f}
.v{font-size:30px;font-weight:800}.mu{color:var(--mu);font-size:12px}
table{width:100%;border-collapse:collapse;font-size:13px}td,th{padding:7px;border-bottom:1px solid var(--bd);text-align:center}th{color:var(--ac)}
.btn{background:#1d2638;color:#fff;border:1px solid #33415c;padding:7px 12px;border-radius:7px;cursor:pointer}.btn.on{background:var(--ok);color:#000}
.row{display:flex;gap:8px;align-items:center;flex-wrap:wrap}input{background:#0b0d12;color:#fff;border:1px solid var(--bd);border-radius:6px;padding:6px;width:110px;text-align:center}
.meter{height:18px;background:#0b0d12;border:1px solid var(--bd);border-radius:6px;position:relative;overflow:hidden;direction:ltr}.mf{height:100%;background:var(--ac)}
</style></head><body><header><b>مراقب الإطارات TPMS PRO V3</b><span id="st" class="mu">...</span></header>
<div class="tabs" id="tabs"><button class="tb on">إطاراتي</button><button class="tb">الماسح</button><button class="tb">الراديو</button><button class="tb">الإعدادات</button></div>
<div class="pg on"><div class="grid"><div class="c" id="t0"></div><div class="c" id="t1"></div><div class="c" id="t2"></div><div class="c" id="t3"></div></div><div class="c" id="t4"></div></div>
<div class="pg"><div class="c"><table><thead><tr><th>ID</th><th>البروتوكول</th><th>الضغط</th><th>الحرارة</th><th>الإشارة</th><th>منذ</th><th>ربط</th></tr></thead><tbody id="sc"></tbody></table></div></div>
<div class="pg"><div class="c"><div class="row" style="justify-content:space-between"><b>الإشارة الحية</b><span id="rl" class="mu"></span></div>
<div class="meter"><div class="mf" id="mf"></div></div><div class="mu" id="rs" style="margin-top:8px;line-height:1.8"></div>
<div class="row" style="margin-top:8px"><span>عرض النطاق</span><button class="btn" onclick="cfg('bw',-1)">-</button><b id="bw"></b><button class="btn" onclick="cfg('bw',1)">+</button>
<span>إزاحة التردد</span><button class="btn" onclick="cfg('foff',-5)">-5</button><b id="fo"></b><button class="btn" onclick="cfg('foff',5)">+5</button>
<span>الحساسية</span><button class="btn" onclick="cfg('d',-1)">-</button><b id="dl"></b><button class="btn" onclick="cfg('d',1)">+</button></div></div></div>
<div class="pg"><div class="c" id="setT"></div><div class="c"><div class="row"><button class="btn" onclick="c('restore=1')">استعادة الحساسات الأصلية</button>
<button class="btn" onclick="c('unit=1')">تبديل الوحدة</button><button class="btn" onclick="if(confirm('إيقاف الواي فاي؟'))c('wifi=0')">إيقاف الواي فاي</button></div></div></div>
<script>
const N=["الأمامي أيسر (FL)","الأمامي أيمن (FR)","الخلفي أيسر (RL)","الخلفي أيمن (RR)","الاحتياطي (SP)"],L=["FL","FR","RL","RR","SP"];let D=null,tab=0;const $=i=>document.getElementById(i);
const e=s=>String(s).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
document.querySelectorAll('.tb').forEach((b,i)=>b.onclick=()=>{tab=i;document.querySelectorAll('.tb').forEach((x,j)=>x.classList.toggle('on',i==j));document.querySelectorAll('.pg').forEach((x,j)=>x.classList.toggle('on',i==j))});
const age=s=>s<60?s+' ث':s<3600?Math.floor(s/60)+' د':Math.floor(s/3600)+' س';
async function c(q){await fetch('/cmd?'+q);up()}
function cfg(k,d){if(!D)return;const r=D.radio;if(k=='bw'){const b=[58,68,81,102,116,135,162,203,232,270,325];let i=b.indexOf(r.bw);i=Math.max(0,Math.min(b.length-1,i+d));c('bw='+b[i])}else if(k=='foff')c('foff='+(r.foff+d));else c('d='+(r.d+d))}
function assign(id){const t=prompt('رقم الإطار:\n1 FL  2 FR  3 RL  4 RR  5 SP');const i=parseInt(t);if(i>=1&&i<=5)c('assign='+(i-1)+'&id='+id)}
async function up(){try{D=await (await fetch('/api')).json();const r=D.radio;
$('st').innerText=(D.link?'متصل بالمستقبل':'الربط مفقود')+' | '+r.f+' MHz | حزم '+r.pk;
D.tires.forEach((t,i)=>{const el=$('t'+i);let cls='c';if(t.has){cls+=t.al&21?' da':t.al?' wa':' ok'}el.className=cls;
el.innerHTML='<div>'+N[i]+'</div>'+(t.has?'<div class="v">'+t.p+' <small>'+D.unit+'</small></div><div class="mu" dir="ltr">'+t.t+'°C | '+t.r+' dBm | '+age(t.age)+'</div><div class="mu">'+e(t.alt)+'</div>':'<div class="mu">بانتظار الإشارة — '+e(t.id)+'</div>')});
$('sc').innerHTML=D.scan.map(s=>'<tr style="color:'+(s.tire>=0?'var(--ok)':'')+'"><td dir="ltr"><b>'+e(s.id)+'</b></td><td>'+e(s.m)+'</td><td>'+s.p+'</td><td>'+s.t+'°</td><td>'+s.r+'</td><td>'+age(s.age)+'</td><td><button class="btn" onclick="assign(\''+e(s.id)+'\')">'+(s.tire>=0?L[s.tire]:'ربط')+'</button></td></tr>').join('');
$('mf').style.width=Math.max(0,Math.min(100,(D.live+110)*100/80))+'%';$('rl').innerText=D.live+' dBm';
$('rs').innerHTML='الضجيج '+r.n+' | العتبة '+r.th+' | مفكوكة '+r.dec+' | مفقودة '+r.lost+' | مرفوضة '+r.rej+' | إعادات تهيئة '+r.rst+'<br>المستقبل '+e(r.fw)+' | تشغيل '+age(r.up);
$('bw').innerText=r.bw+' kHz';$('fo').innerText=(r.foff>0?'+':'')+r.foff+' kHz';$('dl').innerText=r.d+' dB';
$('setT').innerHTML='<b>المعرفات والحدود</b><table><tr><th>الإطار</th><th>المعرف</th><th>أدنى</th><th>أعلى</th><th>حرارة</th></tr>'+D.cfg.map((x,i)=>'<tr><td>'+L[i]+'</td><td dir="ltr">'+e(x.id)+'</td><td>'+x.mn+'</td><td>'+x.mx+'</td><td>'+x.tx+'</td></tr>').join('')+'</table>';
}catch(err){$('st').innerText='غير متصل'}}
up();setInterval(up,1500);
</script></body></html>)HTML";
