#!/usr/bin/env python3
"""Render planned and measured Gazebo pre-dock motion as a standalone SVG."""
import csv
import json
from pathlib import Path

from docking_preview import candidate


def main(output=Path('simulation/output')):
    with (output/'gazebo_trial.csv').open() as f:
        actual=list(csv.DictReader(f))
    planned=candidate(-.16,0.,.9)['rows']
    def sx(x):return 75+470*x
    def sy(y):return 390-700*y
    def pts(rows):return ' '.join(f'{sx(x):.1f},{sy(y):.1f}' for x,y in rows)
    plan_pts=pts((x,y+.16) for x,y,*_ in planned)
    actual_pts=pts((float(r['odom_x_m']),float(r['odom_y_m'])) for r in actual)
    end=actual[-1]
    svg=f'''<svg xmlns="http://www.w3.org/2000/svg" width="1050" height="590" viewBox="0 0 1050 590">
<rect width="1050" height="590" fill="#101827"/>
<g fill="#e5edf8" font-family="sans-serif"><text x="75" y="45" font-size="24">Gazebo pre-dock: planned vs measured odometry</text>
<text x="75" y="73" font-size="14">Laptop simulation, ROS domain 88; stopped before pallet contact</text>
<text x="75" y="525" font-size="14">Blue: quintic + straight path  |  Green: Gazebo odometry</text>
<text x="75" y="548" font-size="14">Final pose ({float(end['odom_x_m']):.3f}, {float(end['odom_y_m']):.3f}) m; no insertion or lift</text></g>
<rect x="75" y="105" width="900" height="390" fill="#182b3d"/>
<path d="M75 390 H975" stroke="#52677c" stroke-dasharray="5 5"/>
<path d="M{sx(1.795):.1f} 105 V495" stroke="#e58d6a" stroke-width="3" stroke-dasharray="7 5"/>
<polyline points="{plan_pts}" fill="none" stroke="#67a9ed" stroke-width="5"/>
<polyline points="{actual_pts}" fill="none" stroke="#6de0c7" stroke-width="3"/>
<circle cx="{sx(.9):.1f}" cy="{sy(.16):.1f}" r="7" fill="#67a9ed"/>
<circle cx="{sx(float(end['odom_x_m'])):.1f}" cy="{sy(float(end['odom_y_m'])):.1f}" r="6" fill="#6de0c7"/>
</svg>'''
    (output/'gazebo_trial.svg').write_text(svg)
    data=[{'t':float(r['time_s']),'x':float(r['odom_x_m']),
           'y':float(r['odom_y_m']),'yaw':float(r['odom_yaw_rad']),
           'state':r['state'],'v':float(r['cmd_v_mps']),
           'w':float(r['cmd_w_rps'])} for r in actual]
    html='''<!doctype html><html lang="vi"><meta charset="utf-8"><title>Gazebo pre-dock replay</title>
<style>body{font:16px system-ui;background:#101827;color:#e5edf8;max-width:1050px;margin:24px auto}
svg{width:100%;background:#182b3d;border-radius:12px}button,input{font:inherit}button{padding:8px 16px}
input{width:78%;vertical-align:middle}.meta{margin:12px 0;color:#bcd2e6}</style>
<h1>Gazebo pre-dock replay</h1><p>Robot mô phỏng dừng trước pallet; không xác minh insertion và không nâng.</p>
<svg viewBox="0 0 1050 560" xmlns="http://www.w3.org/2000/svg">
<path d="M75 390 H975" stroke="#52677c" stroke-dasharray="5 5"/>
<path d="M918.7 75 V490" stroke="#e58d6a" stroke-width="3" stroke-dasharray="7 5"/>
<text x="805" y="65" fill="#e58d6a">mặt pallet</text>
<polyline id="planned" fill="none" stroke="#67a9ed" stroke-width="5"/>
<polyline id="trail" fill="none" stroke="#6de0c7" stroke-width="3"/>
<g id="robot"><rect x="-155" y="-175" width="296" height="350" rx="5" fill="#e5edf8" opacity=".42"/>
<rect x="141" y="-54" width="214" height="37" fill="#f7bf50" opacity=".75"/>
<rect x="141" y="17" width="214" height="37" fill="#f7bf50" opacity=".75"/></g>
</svg><div class="meta" id="meta"></div><button id="play">Phát</button>
<input id="slider" type="range" min="0" max="0" value="0"><script>
const rows=DATA, planned=PLAN;
const sx=x=>75+470*x, sy=y=>390-700*y;
document.getElementById('planned').setAttribute('points',planned.map(p=>`${sx(p[0])},${sy(p[1])}`).join(' '));
const slider=document.getElementById('slider'), play=document.getElementById('play');
slider.max=rows.length-1;let timer=null;
function draw(){const n=+slider.value,r=rows[n];
 document.getElementById('trail').setAttribute('points',rows.slice(0,n+1).map(p=>`${sx(p.x)},${sy(p.y)}`).join(' '));
 document.getElementById('robot').setAttribute('transform',`translate(${sx(r.x)} ${sy(r.y)}) rotate(${-r.yaw*180/Math.PI})`);
 document.getElementById('meta').textContent=`t=${r.t.toFixed(2)} s | ${r.state} | x=${r.x.toFixed(3)} m, y=${r.y.toFixed(3)} m | v=${r.v.toFixed(3)} m/s, ω=${r.w.toFixed(3)} rad/s`;
}slider.addEventListener('input',draw);play.onclick=()=>{if(timer){clearInterval(timer);timer=null;play.textContent='Phát';return;}
 play.textContent='Tạm dừng';timer=setInterval(()=>{if(+slider.value>=rows.length-1){clearInterval(timer);timer=null;play.textContent='Phát';return;}
 slider.value=+slider.value+1;draw();},50);};draw();</script></html>'''
    html=html.replace('DATA',json.dumps(data,separators=(',',':'))).replace(
        'PLAN',json.dumps([[x,y+.16] for x,y,*_ in planned],separators=(',',':')))
    (output/'gazebo_replay.html').write_text(html)


if __name__=='__main__':
    main()
