#!/usr/bin/env python3
"""Build a self-contained loop comparison page; local video copying is explicit and optional."""
import argparse
import html
import json
import math
from pathlib import Path
import re
import shutil
from artifact_manifest import digest, member

STATES = {'EVALUATED', 'FAILED', 'DIAGNOSTIC_COMPLETE', 'PLANNED', 'RUNNING'}


def validate(data):
    if data.get('schema') != 'singularitydog.improvement-index.v1' or not data.get('numbering_scope'):
        raise ValueError('Explicit improvement-index schema and numbering scope required')
    ids, orders = set(), set()
    for loop in data['loops']:
        if type(loop.get('order')) is not int or loop['order'] < 0 or not re.fullmatch(r'L\d{2,}', loop['id']) or loop['id'] in ids or loop['order'] in orders or loop['state'] not in STATES:
            raise ValueError('Duplicate or invalid loop ID/order/state')
        ids.add(loop['id']); orders.add(loop['order'])
        for field in ('title', 'change', 'result', 'comparison_scope'):
            if not isinstance(loop.get(field), str) or not loop[field]:
                raise ValueError('Missing loop explanation: ' + field)
        speed = loop.get('forward_speed_m_s')
        if speed is not None and (isinstance(speed, bool) or not isinstance(speed, (int, float)) or not math.isfinite(speed)):
            raise ValueError('Invalid measured speed')
        counts = loop.get('lift15_counts')
        if counts is not None and (len(counts) != 4 or any(type(x) is not int or x < 0 for x in counts)):
            raise ValueError('Four nonnegative per-foot counts required')
        if loop['state'] in ('PLANNED', 'RUNNING') and (loop.get('video') or speed is not None or counts is not None):
            raise ValueError('Unfinished loop cannot publish final measured results')
    if not ids:
        raise ValueError('At least one loop required')


PAGE = r'''<!doctype html>
<html lang="ja"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>SingularityDog · 改善ループ</title>
<style>
:root{--ink:#15302c;--muted:#58706a;--line:#d7e2dc;--green:#146d53;--bg:#f3f6f0}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--ink);font:15px/1.65 system-ui,sans-serif}main{max-width:1300px;margin:auto;padding:34px 28px 70px}
header{display:flex;justify-content:space-between;gap:20px;align-items:flex-start}.eyebrow{font-size:12px;letter-spacing:.14em;color:var(--green);font-weight:750}h1{font-size:clamp(28px,4vw,46px);letter-spacing:-.04em;margin:8px 0}h2{font-size:20px;margin:0 0 14px}p{margin:8px 0}.muted{color:var(--muted)}.pill{border-radius:30px;padding:5px 12px;display:inline-block;font-size:12px;background:#e3ebe3;white-space:nowrap}.kpis{display:grid;grid-template-columns:repeat(3,1fr);gap:14px;margin:26px 0}.kpi,.panel,.card{border:1px solid var(--line);background:#fff;border-radius:16px;padding:20px}.kpi strong{display:block;font-size:30px}.kpi small{color:var(--muted)}.target{background:#153d32;color:white}.target small{color:#c6dacd}.split{display:grid;grid-template-columns:1.15fr 1fr;gap:20px}.panel{min-width:0}.comparison{display:grid;grid-template-columns:1fr 1fr;gap:18px}.video{aspect-ratio:16/9;background:#172520;border-radius:12px;display:grid;place-items:center;text-align:center;color:#d0ddd2;overflow:hidden;margin:14px 0}video{width:100%;height:100%;background:#10201b}.video p{max-width:80%;font-size:14px}.toolbar{display:flex;gap:10px;flex-wrap:wrap;margin:24px 0 12px}select,button{font:inherit;border:1px solid #bfd0c3;background:white;color:var(--ink);padding:8px 12px;border-radius:9px}button{cursor:pointer}button:hover{border-color:var(--green)}button:focus-visible,select:focus-visible{outline:3px solid #67b6a0}.cards{display:grid;grid-template-columns:repeat(3,1fr);gap:14px}.card{display:flex;flex-direction:column;gap:10px}.card h3{margin:0;font-size:18px}.card p{font-size:14px}.card button{margin-top:auto}.row{display:flex;align-items:center;justify-content:space-between;gap:12px}.FAILED{background:#ffe9df;color:#9d3e19}.EVALUATED{background:#d7f0df;color:#1d6438}.DIAGNOSTIC_COMPLETE{background:#e0ecfc;color:#275891}.PLANNED{background:#edeae5;color:#685c4d}.RUNNING{background:#fcf1c9;color:#7a6221}.meter{height:12px;border-radius:8px;background:#e7eee6;overflow:hidden}.meter span{display:block;height:100%;background:#319275}.numbers{font-variant-numeric:tabular-nums;font-weight:700}.foot{display:grid;grid-template-columns:repeat(4,1fr);gap:6px;margin:12px 0}.foot div{background:#eff5ee;border-radius:8px;padding:10px;text-align:center}.foot strong{display:block;font-size:23px}.foot small{font-size:12px;color:var(--muted)}table{border-collapse:collapse;width:100%;font-size:13px}td,th{padding:10px 6px;border-bottom:1px solid var(--line);text-align:left;vertical-align:top}.notice{border-left:4px solid #85aa83;padding:8px 16px;margin:18px 0;background:#eaf0e6}.note{font-size:12px;color:var(--muted)}.section{margin-top:30px}footer{margin-top:30px;border-top:1px solid var(--line);padding-top:18px;font-size:12px;color:var(--muted)}@media(max-width:900px){.cards{grid-template-columns:1fr 1fr}.split{grid-template-columns:1fr}}@media(max-width:600px){main{padding:20px 14px}.kpis,.cards,.comparison{grid-template-columns:1fr}header{display:block}.kpi strong{font-size:26px}}
</style><main><header><div><div class="eyebrow">SINGULARITYDOG / EXPERIMENT NOTEBOOK</div><h1>一歩ずつ、改善を確かめる。</h1><p class="muted">変更・測定・動画を、同じループ番号でたどる。</p></div><span class="pill" id="updated"></span></header>
<div class="kpis"><div class="kpi"><small>動画評価済み基準・前進平均</small><strong id="baselineSpeed"></strong><small>始動を含む最初の12秒</small></div><div class="kpi target"><small>次の目標・実測平均</small><strong>20.0 <span style="font-size:17px">cm/s</span></strong><small>指令速度と達成速度を区別</small></div><div class="kpi"><small>最新の完了段階</small><strong style="font-size:22px" id="latest"></strong><small>歩行の合格とは別の判定</small></div></div>
<div class="notice" id="scope"></div>
<div class="split"><section class="panel"><h2>改善ループと実測</h2><div style="overflow:auto"><table><thead><tr><th>ループ</th><th>状態</th><th>前進平均</th><th>動画</th></tr></thead><tbody id="summary"></tbody></table></div><p class="note">未測定は「—」。異なる物理・指令条件は単純な改善率に換算しません。</p></section><section class="panel"><h2>基準実験の足上げ・前進</h2><div class="foot" id="feet"></div><p>完了した離床→着地のうち、最高クリアランスが15mm以上の回数。</p><p class="muted">後左脚が主な課題。平均速度だけでは歩行品質を判断しません。</p><div class="meter"><span id="progress"></span></div><p class="note">基準の前進速度 / 20cm/s の目標</p></section></div>
<section class="section"><h2>動画で比較</h2><div class="comparison"><section class="panel"><label for="before">比較元</label><select id="before"></select><div id="beforeView"></div></section><section class="panel"><label for="after">比較先</label><select id="after"></select><div id="afterView"></div></section></div><p class="note">動画は記録した速度のまま再生。比較先の動画がまだない場合、基準動画で代用しません。</p></section>
<section class="section"><div class="row"><h2>ループの記録</h2><select id="filter" aria-label="ループの状態で絞り込む"><option value="ALL">すべて</option><option value="EVALUATED">動画評価済み</option><option value="FAILED">停止・不採用</option><option value="DIAGNOSTIC_COMPLETE">診断完了</option><option value="PLANNED">準備中</option><option value="RUNNING">実行中</option></select></div><div class="cards" id="cards"></div></section><footer id="footer"></footer></main>
<script id="dataset" type="application/json">__DATA__</script><script>
const data=JSON.parse(document.getElementById('dataset').textContent),loops=[...data.loops].sort((a,b)=>a.order-b.order);
const labels={EVALUATED:'動画評価済み',FAILED:'停止・不採用',DIAGNOSTIC_COMPLETE:'診断完了',PLANNED:'準備中',RUNNING:'実行中'};
const esc=x=>String(x??'').replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
const speed=x=>x==null?'—':(x*100).toFixed(2)+' cm/s';const baseline=loops.find(x=>x.state==='EVALUATED');
document.getElementById('updated').textContent='更新 '+data.updated;
document.getElementById('baselineSpeed').textContent=speed(baseline?.forward_speed_m_s);
document.getElementById('latest').textContent=[...loops].reverse().find(x=>['EVALUATED','DIAGNOSTIC_COMPLETE'].includes(x.state))?.title??'未完了';
document.getElementById('scope').textContent=data.numbering_scope;
document.getElementById('summary').innerHTML=loops.map(x=>`<tr><td class="numbers">${esc(x.id)}</td><td><span class="pill ${x.state}">${labels[x.state]}</span></td><td class="numbers">${speed(x.forward_speed_m_s)}</td><td>${x.video_url?'再生可':x.video_available_privately?'ローカルのみ':'なし'}</td></tr>`).join('');
document.getElementById('feet').innerHTML=['前左','前右','後左','後右'].map((n,i)=>`<div><small>${n}</small><strong>${baseline?.lift15_counts?.[i]??'—'}</strong></div>`).join('');
document.getElementById('progress').style.width=Math.min(100,Math.max(0,(baseline?.forward_speed_m_s??0)/.2*100))+'%';
function view(key){const x=loops.find(l=>l.id===document.getElementById(key).value);let v=x.video_url?`<video controls preload="metadata" playsinline src="${esc(x.video_url)}"></video>`:`<p>${(x.video||x.video_available_privately)?'この比較画面には動画を組み込んでいません。':'このループの記録動画はありません。'}<br>${esc(labels[x.state])}</p>`;document.getElementById(key+'View').innerHTML=`<h3>${esc(x.id+' · '+x.title)}</h3><div class="video">${v}</div><p><b>変更：</b>${esc(x.change)}</p><p><b>結果：</b>${esc(x.result)}</p><p class="numbers">前進平均 ${speed(x.forward_speed_m_s)}</p><p class="note">${esc(x.comparison_scope)}</p>`;}
for(const key of ['before','after']){const s=document.getElementById(key);s.innerHTML=loops.map(x=>`<option value="${x.id}">${esc(x.id+' '+x.title)}</option>`).join('');s.value=key==='before'?loops[0].id:loops[loops.length-1].id;s.onchange=()=>view(key);view(key);}
function cards(){const f=document.getElementById('filter').value;document.getElementById('cards').innerHTML=loops.filter(x=>f==='ALL'||x.state===f).map(x=>`<article class="card"><div class="row"><b>${esc(x.id)}</b><span class="pill ${x.state}">${labels[x.state]}</span></div><h3>${esc(x.title)}</h3><p><b>変更</b><br>${esc(x.change)}</p><p><b>結果</b><br>${esc(x.result)}</p><p class="note">${esc(x.comparison_scope)}</p><button data-loop="${x.id}">比較先に表示</button></article>`).join('');document.querySelectorAll('[data-loop]').forEach(b=>b.onclick=()=>{document.getElementById('after').value=b.dataset.loop;view('after');document.getElementById('after').scrollIntoView({behavior:'smooth',block:'center'});});}
document.getElementById('filter').onchange=cards;cards();document.getElementById('footer').textContent=data.notes;
</script></html>'''


def build(index, output, include_videos=False):
    data = json.loads(index.read_text())
    validate(data)
    if output.exists():
        raise FileExistsError('Choose a new dashboard path')
    output.parent.mkdir(parents=True, exist_ok=True)
    for loop in data['loops']:
        loop['video_url'] = None
        if include_videos and loop.get('video'):
            source = member(index.parent, loop['video']['path'])
            if source.suffix.lower() not in ('.mp4', '.webm') or digest(source) != loop['video']['sha256']:
                raise ValueError('Actual local video SHA/type mismatch')
            name = loop['id'] + '-' + digest(source)[:16] + source.suffix.lower()
            target = output.parent / 'media' / name
            target.parent.mkdir(exist_ok=True)
            if target.exists():
                if digest(target) != digest(source):
                    raise ValueError('Existing video differs')
            else:
                shutil.copyfile(source, target)
            loop['video_url'] = 'media/' + name
    payload = json.dumps(data, ensure_ascii=False, allow_nan=False).replace('<', '\\u003c').replace('>', '\\u003e').replace('&', '\\u0026')
    with output.open('x') as stream:
        stream.write(PAGE.replace('__DATA__', payload))
    return {'loops': len(data['loops']), 'videos_included': sum(bool(x['video_url']) for x in data['loops'])}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--index', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--include-videos', action='store_true', help='Explicitly copy only SHA-verified local videos; do not use for third-party assets in public output')
    args = parser.parse_args()
    print(json.dumps(build(args.index, args.output, args.include_videos)))


if __name__ == '__main__':
    main()
