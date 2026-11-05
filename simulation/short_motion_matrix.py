#!/usr/bin/env python3
"""Offline 30/50/100 cm forward-only diagnostic-motion matrix."""
import csv
import math
from pathlib import Path


def case(length, peak_heading_deg):
    heading=math.radians(peak_heading_deg)
    n=200
    ds=length/n
    x=y=0.
    for i in range(n):
        theta=heading*math.sin(math.pi*(i+.5)/n)
        x+=ds*math.cos(theta)
        y+=ds*math.sin(theta)
    max_kappa=abs(heading)*math.pi/length
    # Hypothetical Rmin only; actual robot values must be measured.
    feasible=max_kappa<=2.5
    v=min(.08,.20/max(max_kappa,1e-9))
    return {'distance_m':length,'peak_heading_deg':peak_heading_deg,
            'end_heading_deg':0.,'lateral_displacement_m':round(y,4),
            'forward_displacement_m':round(x,4),'max_kappa_1pm':round(max_kappa,4),
            'estimated_time_s':round(length/v+v/.08,2),
            'result':'ILLUSTRATIVE_FEASIBLE' if feasible else 'SKIP_CURVATURE',
            'reason':'hypothetical kappa limit 2.5 1/m; no hardware validation'}


def main(output=Path('simulation/output')):
    output.mkdir(parents=True,exist_ok=True)
    rows=[case(length,heading) for length in (.3,.5,1.)
          for heading in (0.,5.,-5.,10.,-10.,15.,-15.)]
    with (output/'short_motion_matrix.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    print('short-motion cases:',len(rows),'skipped:',sum(r['result']=='SKIP_CURVATURE' for r in rows))


if __name__=='__main__':
    main()
