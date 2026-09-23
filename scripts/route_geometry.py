#!/usr/bin/env python3
"""Calibrated ordinary-paint geometry and screenshot route, independent of ROS."""
import math
from pathlib import Path
import cv2
import numpy as np
import yaml

SIZE = 4.2
HALF_LENGTH, HALF_WIDTH = .094, .085
AXLE = .0525

def load_contract(path):
    data = yaml.safe_load(Path(path).read_text())
    points = np.array(data['points'], dtype=float)
    if points.ndim != 2 or points.shape[1] != 2 or not np.isfinite(points).all():
        raise ValueError('Expected finite Nx2 point array')
    if len(points) < 3 or np.linalg.norm(points[0] - points[-1]) > 1e-6:
        raise ValueError('Expected ordered route returning to exact birth')
    for delta in np.diff(points, axis=0):
        if np.linalg.norm(delta) < .03:
            raise ValueError('Degenerate route segment')
    return data, points

class PaintGeometry:
    def __init__(self, path):
        source = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        if source is None or source.shape[0] != source.shape[1]:
            raise ValueError('Missing square ground texture')
        self.size = source.shape[0]
        self.paint = (source >= 220).astype(np.uint8)
        # Exception geometry from measured ground paint, not a blanket bypass.
        for axis, line, lateral, width, thickness in [
                ('y', -1.4, 1.7, .32, .055),
                ('x', -.515, 0., .38, .055),
                ('y', -.290, 1.70, .32, .055)]:
            x0,x1,y0,y1 = ((line-thickness,line+thickness,lateral-width,lateral+width)
                          if axis == 'x' else
                          (lateral-width,lateral+width,line-thickness,line+thickness))
            r0,c0 = self.pixel(x1,y1); r1,c1 = self.pixel(x0,y0)
            self.paint[max(0,r0):r1+1,max(0,c0):c1+1] = 0
        for r0,r1,c0,c1 in [(388,1934,6138,6706),(7543,8111,4898,6444)]:
            self.paint[r0:r1+1,c0:c1+1] = 0
        self.display = cv2.resize(source, (850,850), interpolation=cv2.INTER_AREA)

    def pixel(self,x,y):
        return int((.5-x/SIZE)*self.size), int((.5-y/SIZE)*self.size)

    def collision(self,x,y,yaw,margin=0.):
        c,s=math.cos(yaw),math.sin(yaw)
        corners=[]
        for a,b in [(1,1),(1,-1),(-1,-1),(-1,1)]:
            lx=a*(HALF_LENGTH+margin); ly=b*(HALF_WIDTH+margin)
            r,col=self.pixel(x+c*lx-s*ly,y+s*lx+c*ly)
            corners.append((col,r))
        poly=np.array(corners,np.int32)
        x0,y0=poly.min(axis=0); x1,y1=poly.max(axis=0)
        if min(x0,y0)<0 or max(x1,y1)>=self.size: return True
        mask=np.zeros((y1-y0+1,x1-x0+1),np.uint8)
        cv2.fillConvexPoly(mask,poly-[x0,y0],1)
        return bool(np.any(mask & self.paint[y0:y1+1,x0:x1+1]))

def segment_error(p,a,b):
    d=b-a; length=float(np.linalg.norm(d)); unit=d/length
    along=float(np.dot(np.array(p[:2])-a,unit))
    cross=abs(float(np.cross(unit,np.array(p[:2])-a)))
    return along,cross,length

def preflight(geometry,points):
    failures=[]
    for i,(a,b) in enumerate(zip(points[:-1],points[1:])):
        yaw=math.atan2(b[1]-a[1],b[0]-a[0])
        for t in np.linspace(0,1,int(np.linalg.norm(b-a)/.01)+2):
            p=a+t*(b-a)
            if geometry.collision(*p,yaw,.025):
                failures.append((i,float(t),p.tolist(),'translation')); break
        # Chassis moves around the front axle when rotating; check a .06 m
        # extra margin on every possible yaw at a turn.
        for theta in np.linspace(-math.pi,math.pi,73):
            if geometry.collision(*b,theta,.06):
                failures.append((i,1.,b.tolist(),'turn')); break
    return failures

def plot(geometry, points, output, actual=None):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,ax=plt.subplots(figsize=(9,9))
    # Row -> negative world x; column -> negative world y.
    ax.imshow(np.rot90(geometry.display, -1),extent=(-2.1,2.1,-2.1,2.1),cmap='gray')
    ax.plot(points[:,0],points[:,1],'r-',lw=2,label='Required route')
    for i,(a,b) in enumerate(zip(points[:-1],points[1:]),1):
        mid=(a+b)/2
        ax.annotate('',xy=mid+.08*(b-a),xytext=mid-.08*(b-a),arrowprops=dict(color='red',arrowstyle='->',lw=2))
        ax.text(mid[0]+.04,mid[1]+.04,str(i),color='orange',fontsize=12)
    ax.scatter(*points[0],color='lime',s=60,label='Same start / finish')
    if actual is not None:
        actual=np.array(actual); ax.plot(actual[:,0],actual[:,1],color='cyan',lw=1,label='Measured chassis')
    ax.set(xlabel='world x (m)',ylabel='world y (m)',title='Calibrated inner route — signals ignored, ordinary paint forbidden')
    ax.legend(); fig.tight_layout(); fig.savefig(output,dpi=140); plt.close(fig)

if __name__ == '__main__':
    import argparse,json
    p=argparse.ArgumentParser(); p.add_argument('contract'); p.add_argument('texture'); p.add_argument('output')
    args=p.parse_args(); _,points=load_contract(args.contract); geom=PaintGeometry(args.texture)
    failures=preflight(geom,points); plot(geom,points,args.output)
    print(json.dumps({'preflight_failures':failures,'points':points.tolist()}))
    raise SystemExit(bool(failures))
