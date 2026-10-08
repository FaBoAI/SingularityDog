#!/usr/bin/env python3
"""Render a static explanation of the retained STOP-proxy cycle, not Type1 evidence."""
import os
from pathlib import Path

os.environ.setdefault('MPLCONFIGDIR', '/private/tmp/robotdog-can-diagram-mpl')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'docs' / 'images'
OUT.mkdir(parents=True, exist_ok=True)
FONT = FontProperties(fname='/System/Library/Fonts/Hiragino Sans GB.ttc')
fig, ax = plt.subplots(figsize=(16, 13), dpi=160)
fig.patch.set_facecolor('#f5f8fc')
ax.set_facecolor('#f5f8fc')
ax.set_xlim(0, 120)
ax.set_ylim(0, 100)
ax.axis('off')
ink, blue, green, amber, red = '#172d44', '#2064af', '#16816c', '#a46a05', '#c64146'

def text(x, y, value, size=14, color=ink, align='left', weight='normal'):
    ax.text(x, y, value, fontproperties=FONT, fontsize=size, color=color,
            ha=align, va='center', fontweight=weight, linespacing=1.5)

def box(x, y, w, h, color='#ffffff', edge='#d4dee9'):
    ax.add_patch(FancyBboxPatch((x,y),w,h,boxstyle='round,pad=0.5,rounding_size=1.2',
                              facecolor=color,edgecolor=edge,linewidth=1.3))

def arrow(x1, y1, x2, y2, color=blue, width=2):
    ax.add_patch(FancyArrowPatch((x1,y1),(x2,y2),arrowstyle='-|>',
                                mutation_scale=16,color=color,linewidth=width))

text(4,97,'何を送り、何が返るか',27,weight='bold')
text(4,93,'今回の無駆動STOP代理計測・59周期目の原記録',15)

box(4,66,27,23,color='#eaf2fc',edge=blue)
text(17.5,85,'Jetson',21,blue,'center','bold')
text(6,76,'角度・速度／IMU取得\n実モデルで目標を計算\n今回の送信はSTOPへ置換',13)
box(44,69,20,17)
text(54,80,'USB–CAN ×2',17,align='center',weight='bold')
text(54,73,'USB 17B → CANデータ8B',12,align='center')
box(82,79,32,10,color='#f0f9f5',edge=green)
text(98,84,'前側CAN：ID1-6',17,green,'center')
box(82,65,32,10,color='#f0f9f5',edge=green)
text(98,70,'後側CAN：ID7-12',17,green,'center')
arrow(31.5,85,43,85,blue)
text(37,88,'送信',12,blue,'center')
arrow(43,70,31.5,70,green)
text(37,67,'返信',12,green,'center')
arrow(64.5,84,81.5,84,blue)
arrow(81.5,80,64.5,80,green)
arrow(64.5,74,81.5,74,blue)
arrow(81.5,69,64.5,69,green)
text(72.5,88,'Type4 STOP\nType17 電圧読取り',11,blue,'center')
text(72.5,64,'Type2 状態\nType17 電圧値',11,green,'center')

box(4,54,110,7)
text(59,57.5,'1周期：状態取得12件（STOP→状態）＋ 電圧2件 ＋ 推論後STOP12件 ＝ 26要求',13,align='center')

text(4,49,'送信期限と返信時刻を分ける',19,weight='bold')
x0, scale, y = 7, 4.85, 42
ax.plot([x0,x0+21*scale],[y,y],color='#9aabba',linewidth=2)
for t in (0,5,10,15,20):
    x=x0+t*scale
    ax.plot([x,x],[y-0.5,y+0.5],color='#9aabba')
    text(x,y-2,f'{t}ms',11,align='center')
begin, write, reply = .229088, 17.170367, 20.001657
ax.plot([x0+20*scale]*2,[34.5,47],color=red,linewidth=1.4,linestyle='--')
text(x0+20*scale,46.5,'周期開始から20ms',12,red,'right')
ax.scatter([x0+begin*scale,x0+write*scale,x0+reply*scale],[y]*3,
           s=[55,80,80],c=[blue,blue,green],zorder=5)
text(x0+begin*scale,45.5,'入力取得開始\n0.229ms',11,blue)
text(x0+write*scale,45.7,'全軸へのUSB書込み完了\n17.170ms',11,blue,'right')
arrow(x0+begin*scale,36,x0+write*scale,36,blue,3)
text(49,36.9,'入力取得 → 推論 → 全軸HOST送信：16.941ms',15,blue,'center','bold')
text(114,31.4,'最後の返信読取り：20.001657ms\n周期の期限を1.657マイクロ秒超過',13,green,'right')
text(4,31.2,'時刻はJetson側の記録。\nCAN線上の到着時刻は未測定。',11)

box(4,9,110,18,color='#ffffff')
text(7,24,'最後の返信：ID12 → Jetson　Type2 状態（CAN-ID内：停止モード／故障0）',15,green,weight='bold')
fields=[('91 DF','プロトコル角','100.566°'),('7F EF','速度（QDDの値）','-0.02518 rad/s'),
        ('7F FF','推定トルク','-0.000084 N·m'),('01 4A','温度','33.0°C')]
for i,(raw,label,value) in enumerate(fields):
    cx=18.5+i*27
    box(cx-11.8,11,23.6,9,color='#eef8f4',edge='#b9dcd1')
    text(cx,18.5,raw,16,green,'center','bold')
    text(cx,15.3,label,12,align='center')
    text(cx,12.3,value,12,align='center')
text(7,7,'角度は原点・符号・360°枝補正前。電圧、目標角、Kp/Kd、遅延理由はこの返信に入っていません。',11)
text(4,2.5,'実出力時は Type1：目標角・目標速度・Kp・Kd・FFトルクを送信 → 同じ Type2 状態が返信',13,blue)

fig.subplots_adjust(left=.01,right=.99,bottom=.01,top=.99)
for extension in ('png','svg'):
    fig.savefig(OUT/f'20ms-can-cycle-20261007.{extension}',facecolor=fig.get_facecolor())
print(OUT/'20ms-can-cycle-20261007.png')
