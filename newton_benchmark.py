import argparse, json, math, os, statistics, time, re, xml.etree.ElementTree as ET
from pathlib import Path
os.environ['OMP_NUM_THREADS']='1'
os.environ['OPENBLAS_NUM_THREADS']='1'
import numpy as np
import warp as wp
import newton
from newton.solvers import SolverMuJoCo
ROOT=Path()
from benchmark_common import prompt_prices
wp.set_module_options({'enable_backward':False,'fast_math':False,'fuse_fp':False})

@wp.func
def action_bits(world:int,block:int,joint:int):
    x=wp.uint32(20261115) ^ (wp.uint32(world)*wp.uint32(0x9e3779b9)) ^ (wp.uint32(block)*wp.uint32(0x85ebca6b)) ^ (wp.uint32(joint)*wp.uint32(0xc2b2ae35))
    x=x ^ (x >> wp.uint32(16));x=x*wp.uint32(0x7feb352d)
    x=x ^ (x >> wp.uint32(15));x=x*wp.uint32(0x846ca68b)
    return x ^ (x >> wp.uint32(16))

@wp.kernel
def random_targets(counter:wp.array(dtype=int),enabled:wp.array(dtype=int),nact:int,kind:int,scale:float,qi:wp.array(dtype=int),vi:wp.array(dtype=int),nominal:wp.array(dtype=float),tq:wp.array(dtype=float),tv:wp.array(dtype=float),out:wp.array(dtype=float)):
    idx=wp.tid()
    if enabled[0]!=0:
        w=idx//nact;j=idx%nact
        noise=float(action_bits(w,counter[0],j) & wp.uint32(0x00ffffff))*float(0.00000011920928955078125)-float(1.0)
        noise=noise*scale
        value=float(0.0)
        if kind==0:
            value=nominal[j]+float(0.15)*noise
            tq[qi[idx]]=value
        else:
            value=nominal[j]*(float(1.0)+float(0.25)*noise)
            tv[vi[idx]]=value
        out[idx]=value

@wp.kernel
def increment_action(counter:wp.array(dtype=int),enabled:wp.array(dtype=int)):
    if enabled[0]!=0:counter[0]=counter[0]+1

@wp.kernel
def metrics(body:wp.array(dtype=wp.transform), parents:wp.array(dtype=int), children:wp.array(dtype=int), pp:wp.array(dtype=wp.vec3), pc:wp.array(dtype=wp.vec3), gaps:wp.array(dtype=float), overflow:wp.array(dtype=int), accumulated:wp.array(dtype=int)):
    w=wp.tid()
    for k in range(84):
        i=w*84+k
        d=wp.length(wp.transform_point(body[parents[i]],pp[i])-wp.transform_point(body[children[i]],pc[i]))
        gaps[w]=wp.max(gaps[w],d)
    accumulated[w]=accumulated[w] | overflow[w]

@wp.kernel
def penetration(dist:wp.array(dtype=float),count:wp.array(dtype=int),stats:wp.array(dtype=float)):
    i=wp.tid()
    if i<count[0]:
        wp.atomic_max(stats,3,-dist[i])

@wp.kernel
def accumulate_overflow(overflow:wp.array(dtype=int), accumulated:wp.array(dtype=int)):
    w=wp.tid()
    accumulated[w]=accumulated[w] | overflow[w]

@wp.kernel
def contact_metrics(count:wp.array(dtype=int), stats:wp.array(dtype=float)):
    n=float(count[0])
    stats[0]=stats[0]+n
    stats[1]=wp.min(stats[1],n)
    stats[2]=wp.max(stats[2],n)

def template_model(scenario):
    t=newton.ModelBuilder()
    t.default_shape_cfg.density=0.
    t.default_shape_cfg.mu=.8
    t.default_shape_cfg.restitution=0.
    t.rigid_gap=0.
    if scenario=='strandbeest':
        t.add_usd(str(ROOT/'strandbeest.usda'), floating=True, collapse_fixed_joints=False, enable_self_collisions=False, load_visual_shapes=False)
        for j,label in enumerate(t.joint_label):
            if label.endswith('/joint_crossbar_crank'):
                d=t.joint_qd_start[j]
                t.joint_target_kd[d]=200.
                t.joint_target_qd[d]=2*math.pi
                t.joint_target_mode[d]=int(newton.JointTargetMode.VELOCITY)
    else:
        path=ROOT/'anymal.urdf'
        t.add_urdf(str(path),floating=True,collapse_fixed_joints=True,enable_self_collisions=False,joint_damping_mode='passive',xform=wp.transform((0,0,.54),wp.quat_identity()))
        pose=dict(zip(('LF_HAA','LF_HFE','LF_KFE','RF_HAA','RF_HFE','RF_KFE','LH_HAA','LH_HFE','LH_KFE','RH_HAA','RH_HFE','RH_KFE'),(.03,.4,-.8,-.03,.4,-.8,.03,-.4,.8,-.03,-.4,.8)))
        for j,label in enumerate(t.joint_label):
            name=label.rsplit('/',1)[-1]
            if name in pose:
                q,d=t.joint_q_start[j],t.joint_qd_start[j]
                t.joint_q[q]=pose[name]
                t.joint_target_q[q]=pose[name]
                t.joint_target_ke[d]=200.
                t.joint_target_kd[d]=10.
                t.joint_target_mode[d]=int(newton.JointTargetMode.POSITION)
    return t

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--scenario',choices=['anymal','strandbeest'],required=True)
    ap.add_argument('--worlds',type=int,default=1)
    ap.add_argument('--steps',type=int,default=3000)
    ap.add_argument('--repeats',type=int,default=3)
    ap.add_argument('--profile',choices=['default','stiff'],default='default')
    ap.add_argument('--output',required=True)
    ap.add_argument('--assets',type=Path,required=True)
    ap.add_argument('--cpu-price',type=float)
    ap.add_argument('--gpu-price',type=float)
    ap.add_argument('--action-scale',type=float,default=1.)
    a=ap.parse_args()
    global ROOT
    ROOT=a.assets.resolve()
    cpu_price,gpu_price=prompt_prices(a.cpu_price,a.gpu_price)
    if a.worlds<1 or a.steps<10 or a.steps%10 or a.repeats<1 or not math.isfinite(a.action_scale) or not 0<=a.action_scale<=1:raise ValueError('positive counts and multiple-of-ten steps required')
    wp.config.kernel_cache_dir=str(ROOT/'warp-cache')
    wp.config.enable_backward=False
    wp.init()
    t=template_model(a.scenario)
    b=newton.ModelBuilder()
    b.default_shape_cfg.mu=.8
    b.default_shape_cfg.restitution=0.
    b.rigid_gap=0.
    b.replicate(t,a.worlds)
    b.add_ground_plane()
    model=b.finalize(device='cuda:0')
    solver=SolverMuJoCo(model, njmax=512 if a.scenario=='strandbeest' else 128,nconmax=64,iterations=100,ls_iterations=50,solver='newton',integrator='implicitfast',cone='pyramidal',tolerance=1e-8,enable_sleeping=False,kinematic_switchable_roots=False,use_mujoco_contacts=True)
    for name,values in [('geom_solref',[.01 if a.scenario=='anymal' else .002,1.]),('geom_solimp',[.99,.999,.001,.5,2.])]:
        arr=getattr(solver.mjw_model,name)
        data=arr.numpy();data[...]=values;arr.assign(data)
        getattr(solver.mj_model,name)[:]=values
    if a.profile=='stiff':
        for name,values in [('eq_solref',[.002,1.]),('eq_solimp',[.99,.999,.001,.5,2.])]:
            arr=getattr(solver.mjw_model,name)
            data=arr.numpy()
            data[...]=values
            arr.assign(data)
            getattr(solver.mj_model,name)[:]=values
    print('MODEL_READY',json.dumps(dict(nq=solver.mj_model.nq,nv=solver.mj_model.nv,neq=solver.mj_model.neq,mass=sum(t.body_mass),bodies=t.body_count,shapes=t.shape_count)),flush=True)
    states=[model.state(),model.state()]
    control=model.control()
    names=['LF_HAA','LF_HFE','LF_KFE','RF_HAA','RF_HFE','RF_KFE','LH_HAA','LH_HFE','LH_KFE','RH_HAA','RH_HFE','RH_KFE'] if a.scenario=='anymal' else ['joint_crossbar_crank']
    nominal_values=[.03,.4,-.8,-.03,.4,-.8,.03,-.4,.8,-.03,-.4,.8] if a.scenario=='anymal' else [2*math.pi]
    nact=len(names);qindices=[];vindices=[]
    tqstart=model.joint_target_q_start.numpy();vdstart=model.joint_qd_start.numpy()
    for w in range(a.worlds):
        labels={label.rsplit('/',1)[-1]:w*t.joint_count+j for j,label in enumerate(t.joint_label)}
        for name in names:
            j=labels[name];qindices.append(int(tqstart[j]));vindices.append(int(vdstart[j]))
    qi=wp.array(qindices,dtype=int);vi=wp.array(vindices,dtype=int);nominal=wp.array(nominal_values,dtype=float)
    counter=wp.zeros(1,dtype=int);enabled=wp.zeros(1,dtype=int);action_out=wp.zeros(a.worlds*nact,dtype=float)
    def apply_actions():
        wp.launch(random_targets,dim=a.worlds*nact,inputs=[counter,enabled,nact,0 if a.scenario=='anymal' else 1,a.action_scale,qi,vi,nominal,control.joint_target_q,control.joint_target_qd,action_out])
        wp.launch(increment_action,dim=1,inputs=[counter,enabled])
    initial_tq=wp.clone(control.joint_target_q);initial_tv=wp.clone(control.joint_target_qd)
    dt=.001 if a.scenario=='strandbeest' else .002
    initial_q,initial_v=wp.clone(model.joint_q),wp.clone(model.joint_qd)
    def reset():
        counter.zero_();wp.copy(control.joint_target_q,initial_tq);wp.copy(control.joint_target_qd,initial_tv)
        solver.reset(states[0],flags=0)
        for s in states:
            wp.copy(s.joint_q,initial_q);wp.copy(s.joint_qd,initial_v)
            s.clear_forces()
            newton.eval_fk(model,s.joint_q,s.joint_qd,s)
    def step():
        states[0].clear_forces()
        solver.step(states[0],states[1],control,None,dt)
        states[0],states[1]=states[1],states[0]
    reset()
    body_initial=states[0].body_q.numpy()
    parents=[];children=[];pp=[];pc=[];base_ids=[]
    if a.scenario=='strandbeest':
        points=json.loads((ROOT/'anchors.json').read_text())
        for w in range(a.worlds):
            idx={label.rsplit('/',1)[-1]:w*t.body_count+i for i,label in enumerate(t.body_label)}
            base_ids.append(idx['crossbar'])
            for p,c,point in points:
                ip,ic=idx[p.rsplit('/',1)[-1]],idx[c.rsplit('/',1)[-1]]
                parents.append(ip);children.append(ic)
                for target,i in [(pp,ip),(pc,ic)]:
                    transform=wp.transform(wp.vec3(*body_initial[i,:3]),wp.quat(*body_initial[i,3:]))
                    target.append(wp.transform_point(wp.transform_inverse(transform),wp.vec3(*point)))
    else: base_ids=[w*t.body_count for w in range(a.worlds)]
    arrays=[wp.array(parents,dtype=int),wp.array(children,dtype=int),wp.array(pp,dtype=wp.vec3),wp.array(pc,dtype=wp.vec3)]
    gaps=wp.zeros(a.worlds,dtype=float)
    overflows=wp.zeros(a.worlds,dtype=int)
    stats=wp.array([0.,float('inf'),0.,0.],dtype=float)
    for _ in range(2):step()
    wp.synchronize()
    with wp.ScopedCapture() as capture:
        apply_actions()
        for _ in range(10):step()
    graph=capture.graph
    reset()
    for _ in range(110 if a.scenario=='anymal' else 10):wp.capture_launch(graph)
    wp.synchronize()
    if a.scenario=='anymal':
        wp.copy(initial_q,states[0].joint_q);wp.copy(initial_v,states[0].joint_qd)
    enabled.assign([1])
    print('WARMUP_DONE',flush=True)
    seconds=[]
    for r in range(a.repeats):
        reset();wp.synchronize()
        start=time.perf_counter()
        for _ in range(a.steps//10):wp.capture_launch(graph)
        wp.synchronize();seconds.append(time.perf_counter()-start)
        print('TRIAL',r+1,seconds[-1],flush=True)
    reset()
    with wp.ScopedCapture() as diagnostic:
        apply_actions()
        for _ in range(10):
            step()
            if a.scenario=='strandbeest':
                newton.eval_fk(model,states[0].joint_q,states[0].joint_qd,states[0])
                wp.launch(metrics,dim=a.worlds,inputs=[states[0].body_q,*arrays,gaps,solver.mjw_data.overflow,overflows])
            wp.launch(accumulate_overflow,dim=a.worlds,inputs=[solver.mjw_data.overflow,overflows])
            wp.launch(penetration,dim=solver.mjw_data.contact.dist.size,inputs=[solver.mjw_data.contact.dist,solver.mjw_data.nacon,stats])
            wp.launch(contact_metrics,dim=1,inputs=[solver.mjw_data.nacon,stats])
    for block in range(a.steps//10):
        wp.capture_launch(diagnostic.graph)
        if (block+1)%100==0:
            wp.synchronize();print('VALIDATION',10*(block+1),flush=True)
    wp.synchronize()
    newton.eval_fk(model,states[0].joint_q,states[0].joint_qd,states[0])
    body=states[0].body_q.numpy();q=states[0].joint_q.numpy();v=states[0].joint_qd.numpy()
    stat=stats.numpy();gap=float(gaps.numpy().max());overflow=int(np.bitwise_or.reduce(overflows.numpy()))
    finite=bool(np.isfinite(q).all() and np.isfinite(v).all() and np.isfinite(body).all())
    walked=np.linalg.norm(body[base_ids,:2]-body_initial[base_ids,:2],axis=1)
    result=dict(engine='newton-mujoco',scenario=a.scenario,profile=a.profile,worlds=a.worlds,steps=a.steps,dt=dt,seconds=seconds,aggregate_fps=a.worlds*a.steps/statistics.median(seconds),batch_steps_per_second=a.steps/statistics.median(seconds),mean_contacts_per_world=float(stat[0]/a.steps/a.worlds),min_batch_average_contacts=float(stat[1]/a.worlds),max_batch_average_contacts=float(stat[2]/a.worlds),max_loop_gap_m=gap,overflow_bitmask=overflow,finite=finite,max_ground_penetration_m=float(stat[3]),contact_solref=[.01 if a.scenario=='anymal' else .002,1.],contact_solimp=[.99,.999,.001,.5,2.],base_height_min=float(body[base_ids,2].min()),base_height_max=float(body[base_ids,2].max()),walked_min_m=float(walked.min()),walked_max_m=float(walked.max()),max_abs_velocity=float(np.abs(v).max()),mass_per_world=float(sum(t.body_mass)),bodies_per_world=t.body_count,shapes_per_world=t.shape_count,mujoco_nv=solver.mj_model.nv,mujoco_equalities=solver.mj_model.neq,eq_solref=solver.mjw_model.eq_solref.numpy()[0,:1].tolist(),quality_pass=finite and overflow==0 and (gap<.001 if a.scenario=='strandbeest' else bool(np.all(body[base_ids,2]>-.1))))
    import hashlib
    result['state_sha256']=hashlib.sha256(q.tobytes()+v.tobytes()).hexdigest()
    result.update(cpu_price_usd=cpu_price,gpu_price_usd=gpu_price,fps_per_usd=result['aggregate_fps']/gpu_price)
    result.update(action_scale=a.action_scale,action_seed=20261115,action_decimation=10,action_kind='bounded_pd_targets',action_dofs=nact,action_updates=int(counter.numpy()[0]),last_action_checksum=float(np.sum(action_out.numpy(),dtype=np.float64)),last_actions=action_out.numpy()[:min(24,a.worlds*nact)].tolist(),base_height_mean=float(body[base_ids,2].mean()),policy_inference=False)
    Path(a.output).write_text(json.dumps(result,indent=2)+'\n')
    print('RESULT '+json.dumps(result),flush=True)
    print(f"Newton: {result['aggregate_fps']:,.2f} FPS, {result['fps_per_usd']:,.2f} FPS/USD",flush=True)
if __name__=='__main__':main()
