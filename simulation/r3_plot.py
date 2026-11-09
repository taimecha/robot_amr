"""Default user view: exactly two fork-root trajectories, no extra odom/base lines."""
import matplotlib
matplotlib.use('Agg')
from matplotlib.figure import Figure
import math
from matplotlib.patches import Polygon, Circle, FancyArrowPatch
from r3_core import polygons,rectangle,base_for_fork,fork_for_base,compose


def draw_two_paths(ax,planned,actual,target,g,actual_label='Thực tế (Gazebo)',*,scene=None):
    """Project odom paths into the fixed scene for display only.

    A case's world->odom registration changes with the initial vehicle pose.
    The physical pallet stays at scene['pallet_world'], including when a
    measured planner target differs from it. No controller input is changed.
    Callers without a scene retain the original odom diagnostic view.
    """
    ax.clear()
    initial_fork=fork_for_base((0.,0.,0.),g)
    first=planned[0] if planned else initial_fork
    start=(*first[:2],first[2] if len(first)>2 else initial_fork[2])
    frame='odom'
    if scene is not None:
        world_from_odom=scene['base_world']
        def project(p):
            pose=compose(world_from_odom,(*p[:2],p[2] if len(p)>2 else 0.))
            return pose if len(p)>2 else pose[:2]
        start=project(start)
        planned=[project(p) for p in planned]
        actual=[project(p) for p in actual]
        target=scene['pallet_world']
        frame='world'
    if planned:ax.plot([p[0] for p in planned],[p[1] for p in planned],color='#246ac5',lw=2,label='Bộ tạo quỹ đạo')
    if actual:ax.plot([p[0] for p in actual],[p[1] for p in actual],color='#ed7c22',lw=2,ls='--',label=actual_label)
    if target is not None:
        body,tines,legs=polygons(base_for_fork(start,g),target,g)
        for leg in legs:ax.add_patch(Polygon(leg,color='#785c47',alpha=.9))
        ax.add_patch(Polygon(rectangle(target,(0.,g['pallet_length'],-g['pallet_width']/2,g['pallet_width']/2)),fill=False,ec='#444',gid='pallet_outline'))
        ax.add_patch(Circle(start[:2],radius=.008,color='#555',gid='initial_fork_root'))
        if scene is not None:
            ax.add_patch(Polygon(body,fill=False,ec='#777',lw=1.2,alpha=.8,gid='initial_base'))
            for tine in tines:ax.add_patch(Polygon(tine,fill=False,ec='#777',lw=1.,alpha=.8))
            end=(start[0]+.20*math.cos(start[2]),start[1]+.20*math.sin(start[2]))
            ax.add_patch(FancyArrowPatch(start[:2],end,arrowstyle='->',mutation_scale=12,color='#555'))
            base=base_for_fork(start,g)
            ax.annotate('Xe lúc đầu',base[:2],xytext=(0,12),textcoords='offset points',ha='center',color='#555',fontsize=8)
    if not planned and not actual:
        ax.text(.5,.86,'Ca hiện tại chưa có quỹ đạo\nBấm “Tạo quỹ đạo” để xem đường',transform=ax.transAxes,ha='center',color='#555')
    ax.relim();ax.autoscale_view()
    ax.set_aspect('equal',adjustable='datalim');ax.set_xlabel(frame+' X (m)');ax.set_ylabel(frame+' Y (m)')
    ax.set_title('Quỹ đạo tâm gốc càng: dự kiến và thực tế');ax.grid(alpha=.2)
    if planned or actual:ax.legend(loc='best')


def save_two_paths(out,planned,actual,target,g,actual_label='Thực tế (Gazebo)',name='trajectory',*,scene=None):
    fig=Figure(figsize=(11,5));ax=fig.subplots();draw_two_paths(ax,planned,actual,target,g,actual_label,scene=scene)
    fig.tight_layout()
    for ext in ('png','svg'):fig.savefig(out/f'{name}.{ext}',dpi=150)
