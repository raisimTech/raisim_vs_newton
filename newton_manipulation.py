"""GPU implementation of the public Kinova/Robotiq manipulation workload."""
import argparse
import json
import os
import statistics
import time
from pathlib import Path
os.environ['OMP_NUM_THREADS'] = '1'
os.environ['OPENBLAS_NUM_THREADS'] = '1'
import numpy as np
import warp as wp
import newton
from newton.solvers import SolverMuJoCo
from manipulation_common import ARM, DRIVER, FOLLOWERS, SIGNS, KP, KD, HOVER, CYCLE, object_spec, reset_period, bits
wp.set_module_options({'enable_backward': False, 'fast_math': False, 'fuse_fp': False})


@wp.func
def random_bits(world: int, block: int, channel: int):
    x = wp.uint32(20261117) ^ (wp.uint32(world) * wp.uint32(0x9e3779b9)) ^ (wp.uint32(block) * wp.uint32(0x85ebca6b)) ^ (wp.uint32(channel) * wp.uint32(0xc2b2ae35))
    x = (x ^ (x >> wp.uint32(16))) * wp.uint32(0x7feb352d)
    x = (x ^ (x >> wp.uint32(15))) * wp.uint32(0x846ca68b)
    return x ^ (x >> wp.uint32(16))


@wp.func
def uniform(w: int, block: int, channel: int):
    return float(random_bits(w, block, channel) & wp.uint32(0xffffff)) / float(16777216.0)


@wp.kernel
def reset_mask(counter: wp.array(dtype=int), periods: wp.array(dtype=int), mask: wp.array(dtype=wp.bool)):
    w = wp.tid()
    mask[w] = counter[0] % periods[w] == 0


@wp.kernel
def commands(counter: wp.array(dtype=int), periods: wp.array(dtype=int), delays: wp.array(dtype=int),
             table: wp.array2d(dtype=float), qi: wp.array2d(dtype=int), vi: wp.array2d(dtype=int),
             object_q: wp.array(dtype=int), heights: wp.array(dtype=float),
             q: wp.array(dtype=float), tq: wp.array(dtype=float), tv: wp.array(dtype=float),
             force: wp.array(dtype=float), checksum: wp.array(dtype=wp.float64), reset_counts: wp.array(dtype=wp.int64), record: int):
    w = wp.tid()
    block = counter[0]
    age = block % periods[w]
    phase = wp.clamp(age - delays[w], 0, 2149)
    for j in range(7):
        tq[qi[w, j]] = table[phase, j]
        tv[vi[w, j]] = table[phase, j + 7]
    torque = float(.25) * (float(2.0) * uniform(w, block, 50) - float(1.0))
    force[vi[w, 6]] = torque
    if record != 0:
        checksum[w] = checksum[w] + wp.float64(torque)
        if age == 0 and block > 0:
            reset_counts[w] += wp.int64(1)
    if age == 0:
        epoch = block // periods[w]
        i = object_q[w]
        q[i] = float(.42) + float(.004) * (uniform(w, epoch, 40) - float(.5))
        q[i+1] = float(-.18) + float(.004) * (uniform(w, epoch, 41) - float(.5))
        q[i+2] = heights[w]
        yaw = float(6.283185307179586) * uniform(w, epoch, 42)
        q[i+3] = float(0.0)
        q[i+4] = float(0.0)
        q[i+5] = wp.sin(yaw / float(2.0))
        q[i+6] = wp.cos(yaw / float(2.0))


@wp.kernel
def increment(counter: wp.array(dtype=int)):
    counter[0] += 1


@wp.kernel
def diagnostics(q: wp.array(dtype=float), v: wp.array(dtype=float), bodies: wp.array(dtype=wp.transform),
                driver: wp.array(dtype=int), follower: wp.array2d(dtype=int), signs: wp.array(dtype=float),
                object_body: wp.array(dtype=int), heights: wp.array(dtype=float),
                overflow: wp.array(dtype=int), accumulated: wp.array(dtype=int),
                mimic: wp.array(dtype=float), lifted: wp.array(dtype=int), bad: wp.array(dtype=int)):
    w = wp.tid()
    accumulated[w] = accumulated[w] | overflow[w]
    for j in range(5):
        mimic[w] = wp.max(mimic[w], wp.abs(q[follower[w,j]] - signs[j] * q[driver[w]]))
    if wp.transform_get_translation(bodies[object_body[w]])[2] > heights[w] + float(.1):
        lifted[w] += 1
    nq = q.shape[0] // driver.shape[0]
    nv = v.shape[0] // driver.shape[0]
    for j in range(nq):
        if not wp.isfinite(q[w*nq+j]):
            bad[w] = 1
    for j in range(nv):
        if not wp.isfinite(v[w*nv+j]):
            bad[w] = 1


@wp.kernel
def contact_stats(count: wp.array(dtype=int), total: wp.array(dtype=wp.int64)):
    total[0] += wp.int64(count[0])


@wp.kernel
def object_contacts(count: wp.array(dtype=int), shape0: wp.array(dtype=int), shape1: wp.array(dtype=int),
                    shape_body: wp.array(dtype=int), body_world: wp.array(dtype=int),
                    objects: wp.array(dtype=int), pads: wp.array2d(dtype=int),
                    flags: wp.array(dtype=int), totals: wp.array(dtype=wp.int64)):
    i = wp.tid()
    if i < count[0] and i < shape0.shape[0]:
        a = shape_body[shape0[i]]
        b = shape_body[shape1[i]]
        if a >= 0 and b >= 0:
            w = body_world[a]
            other = int(-1)
            if a == objects[w]:
                other = b
            elif b == objects[w]:
                other = a
            if other >= 0:
                wp.atomic_add(totals, 0, wp.int64(1))
                if other == pads[w,0]:
                    wp.atomic_or(flags,w,1)
                if other == pads[w,1]:
                    wp.atomic_or(flags,w,2)


@wp.kernel
def bilateral_stats(flags: wp.array(dtype=int), totals: wp.array(dtype=wp.int64)):
    w = wp.tid()
    if flags[w] == 3:
        wp.atomic_add(totals,1,wp.int64(1))


def run(args):
    wp.init()
    if not wp.get_device('cuda:0').is_cuda:
        raise RuntimeError('CUDA GPU required')
    wp.config.enable_backward = False
    t = newton.ModelBuilder()
    t.rigid_gap = 0.
    t.default_shape_cfg.density = 0.
    t.default_shape_cfg.mu = .8
    t.default_shape_cfg.restitution = 0.
    t.add_urdf(str(args.assets / 'robotiq.urdf'), floating=False, collapse_fixed_joints=True,
               enable_self_collisions=False, joint_damping_mode='passive')
    labels = {s.rsplit('/', 1)[-1]: j for j, s in enumerate(t.joint_label)}
    if any(s not in labels for s in (*ARM, DRIVER, *FOLLOWERS)):
        raise RuntimeError('Missing robot joints')
    for name, home, kp, kd in zip((*ARM, DRIVER), (*HOVER, 0.), KP, KD):
        j = labels[name]
        q, v = t.joint_q_start[j], t.joint_qd_start[j]
        t.joint_q[q] = home
        t.joint_target_q[q] = home
        t.joint_target_ke[v] = kp
        t.joint_target_kd[v] = kd
        t.joint_target_mode[v] = int(newton.JointTargetMode.POSITION_VELOCITY)
    for s, label in enumerate(t.shape_label):
        body = t.shape_body[s]
        name = t.body_label[body] if body >= 0 else ''
        t.shape_material_mu[s] = 1. if 'finger_tip_link' in name else .5
    t.add_shape_box(-1, xform=wp.transform((.43, 0., .30), wp.quat_identity()), hx=.29, hy=.37, hz=.02, label='table')
    b = newton.ModelBuilder()
    b.rigid_gap = 0.
    objects, heights = [], []
    for w in range(args.worlds):
        b.begin_world()
        b.add_builder(t)
        kind, (x, y, z) = object_spec(w)
        height = .32 + (x if kind == 1 else z) / 2
        heights.append(height)
        body = b.add_body(xform=wp.transform((.42, -.18, height), wp.quat_identity()), label='object')
        objects.append(body)
        variants = []
        for variant_kind in range(3):
            variant = newton.ModelBuilder()
            variant.rigid_gap = 0.
            vb = variant.add_body()
            volume = x*y*z if variant_kind == 0 else (4/3*np.pi*(x/2)**3 if variant_kind == 1 else np.pi*(x/2)**2*z)
            cfg = newton.ModelBuilder.ShapeConfig(density=.1/volume, mu=.5, restitution=0.)
            if variant_kind == 0:
                variant.add_shape_box(vb, hx=x/2, hy=y/2, hz=z/2, cfg=cfg)
            elif variant_kind == 1:
                variant.add_shape_sphere(vb, radius=x/2, cfg=cfg)
            else:
                variant.add_shape_cylinder(vb, radius=x/2, half_height=z/2, cfg=cfg)
            variants.append(variant)
        b.add_body_variants(body, variants, initial=0)
        b.add_ground_plane(cfg=newton.ModelBuilder.ShapeConfig(mu=.8, restitution=0.), label='ground')
        b.end_world()
    model = b.finalize(device='cuda:0')
    solver = SolverMuJoCo(model, njmax=256, nconmax=128, iterations=100, ls_iterations=50,
                          solver='newton', integrator='implicitfast', cone='elliptic', tolerance=1e-8,
                          enable_sleeping=False, kinematic_switchable_roots=False, use_mujoco_contacts=False)
    flags = model.select_body_variants(wp.array([object_spec(w)[0] for w in range(args.worlds)],dtype=int), bodies=wp.array(objects,dtype=int))
    solver.notify_model_changed(flags)
    mapping = solver.mjc_geom_to_newton_shape.numpy()[0]
    priority = solver.mjw_model.geom_priority.numpy()
    for geom, shape in enumerate(mapping):
        if shape >= 0 and model.shape_label[shape].rsplit('/',1)[-1] in ('table','ground'):
            priority[geom] = 1
    solver.mjw_model.geom_priority.assign(priority)
    solver.mj_model.geom_priority[:] = priority
    for name, value in [('geom_solref', [.002, 1.]), ('geom_solimp', [.99, .999, .001, .5, 2.]),
                        ('eq_solref', [.002, 1.]), ('eq_solimp', [.9999, .99999, .00001, .5, 2.])]:
        arr = getattr(solver.mjw_model, name)
        data = arr.numpy(); data[...] = value; arr.assign(data)
        getattr(solver.mj_model, name)[:] = value
    pipeline = newton.CollisionPipeline(model)
    collision_contacts = pipeline.contacts()
    diagnostic_observables = solver.observables(kinds={solver.ObservableKind.OVERFLOW})
    states = [model.state(), model.state()]
    control = model.control()
    qstart, vstart = model.joint_q_start.numpy(), model.joint_qd_start.numpy()
    target_start = model.joint_target_q_start.numpy()
    qi, vi, followers, driver, oq = [], [], [], [], []
    for w in range(args.worlds):
        offset = w * (t.joint_count + 1)
        qi.append([int(target_start[offset+labels[s]]) for s in (*ARM, DRIVER)])
        vi.append([int(vstart[offset+labels[s]]) for s in (*ARM, DRIVER)])
        driver.append(int(qstart[offset+labels[DRIVER]]))
        followers.append([int(qstart[offset+labels[s]]) for s in FOLLOWERS])
        oq.append(int(qstart[offset+t.joint_count]))
    periods = [reset_period(w) for w in range(args.worlds)]
    counter = wp.zeros(1, dtype=int)
    mask = wp.zeros(args.worlds+1, dtype=wp.bool)
    checksum = wp.zeros(args.worlds, dtype=wp.float64)
    common = [counter, wp.array(periods, dtype=int), wp.array([bits(w,0,31)%101 for w in range(args.worlds)], dtype=int),
              wp.array(np.loadtxt(args.assets/'trajectory.txt', dtype=np.float32), dtype=float),
              wp.array(qi,dtype=int), wp.array(vi,dtype=int), wp.array(oq,dtype=int), wp.array(heights,dtype=float)]
    overflow = wp.zeros(args.worlds,dtype=int)
    mimic = wp.zeros(args.worlds,dtype=float)
    lifted = wp.zeros(args.worlds,dtype=int)
    bad = wp.zeros(args.worlds,dtype=int)
    contacts = wp.zeros(1,dtype=wp.int64)
    diagnostic_args = [wp.array(driver,dtype=int), wp.array(followers,dtype=int), wp.array(SIGNS,dtype=float), wp.array(objects,dtype=int), common[-1]]
    pads = []
    for w in range(args.worlds):
        names = {label.rsplit('/',1)[-1]:w*(t.body_count+1)+j for j,label in enumerate(t.body_label)}
        pads.append([names['robotiq_85_left_finger_tip_link'],names['robotiq_85_right_finger_tip_link']])
    pad_array = wp.array(pads,dtype=int)
    object_array = wp.array(objects,dtype=int)
    contact_flags = wp.zeros(args.worlds,dtype=int)
    object_totals = wp.zeros(2,dtype=wp.int64)
    reset_counts = wp.zeros(args.worlds,dtype=wp.int64)
    def step_block(diagnostic=False):
        wp.launch(reset_mask, dim=args.worlds, inputs=[counter,common[1],mask])
        solver.reset(states[0],world_mask=mask)
        control.joint_f.zero_()
        wp.launch(commands, dim=args.worlds, inputs=[*common,states[0].joint_q,control.joint_target_q,control.joint_target_qd,control.joint_f,checksum,reset_counts,int(diagnostic)])
        for _ in range(10):
            states[0].clear_forces()
            newton.eval_fk(model,states[0].joint_q,states[0].joint_qd,states[0])
            pipeline.collide(states[0],collision_contacts)
            solver.step(states[0],states[1],control,collision_contacts,.001,observables=diagnostic_observables if diagnostic else None)
            states[0],states[1] = states[1],states[0]
            if diagnostic:
                contact_flags.zero_()
                wp.launch(object_contacts,dim=collision_contacts.rigid_contact_max,inputs=[collision_contacts.rigid_contact_count,collision_contacts.rigid_contact_shape0,collision_contacts.rigid_contact_shape1,model.shape_body,model.body_world,object_array,pad_array,contact_flags,object_totals])
                wp.launch(bilateral_stats,dim=args.worlds,inputs=[contact_flags,object_totals])
                newton.eval_fk(model,states[0].joint_q,states[0].joint_qd,states[0])
                wp.launch(diagnostics,dim=args.worlds,inputs=[states[0].joint_q,states[0].joint_qd,states[0].body_q,*diagnostic_args,diagnostic_observables.overflow,overflow,mimic,lifted,bad])
                wp.launch(contact_stats,dim=1,inputs=[solver.mjw_data.nacon,contacts])
        wp.launch(increment,dim=1,inputs=[counter])
    step_block(True)
    wp.synchronize()
    with wp.ScopedCapture() as timed:
        step_block()
    with wp.ScopedCapture() as diagnostic:
        step_block(True)
    counter.zero_()
    for _ in range(10):
        wp.capture_launch(timed.graph)
    wp.synchronize()
    seconds = []
    for _ in range(args.repeats):
        counter.zero_(); wp.synchronize()
        start = time.perf_counter()
        for _ in range(args.steps//10):
            wp.capture_launch(timed.graph)
        wp.synchronize()
        seconds.append(time.perf_counter()-start)
    object_totals.zero_(); reset_counts.zero_()
    counter.zero_(); checksum.zero_(); overflow.zero_(); mimic.zero_(); lifted.zero_(); bad.zero_(); contacts.zero_()
    for _ in range(args.steps//10):
        wp.capture_launch(diagnostic.graph)
    wp.synchronize()
    finite = not bool(bad.numpy().any())
    max_mimic = float(mimic.numpy().max())
    over = int(np.bitwise_or.reduce(overflow.numpy()))
    result = dict(solver_substeps=1, integration_dt=.001, backend='newton', model='robotiq', worlds=args.worlds, steps=args.steps, seconds=seconds,
                  aggregate_fps=args.worlds*args.steps/statistics.median(seconds), finite=finite,
                  overflow_bitmask=over, max_mimic_error_rad=max_mimic, lifted_samples=int(lifted.numpy().sum()),
                  object_contact_samples=int(object_totals.numpy()[0]), bilateral_grasp_samples=int(object_totals.numpy()[1]),
                  batch_contact_samples=int(contacts.numpy()[0]), force_checksum=float(checksum.numpy().astype(np.float64).sum()),
                  resets=int(reset_counts.numpy().sum()),
                  quality_pass=finite and over == 0 and max_mimic < .01 and (args.steps < 21500 or (int(object_totals.numpy()[0]) > 0 and int(object_totals.numpy()[1]) > 0 and int(lifted.numpy().sum()) > 0)),
                  collision_pipeline='newton', solver='SolverMuJoCo')
    print('RESULT '+json.dumps(result),flush=True)
    if not result['quality_pass']:
        raise RuntimeError('Manipulation trajectory failed validation')


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--assets',type=Path,required=True)
    p.add_argument('--worlds',type=int,default=1)
    p.add_argument('--steps',type=int,default=30000)
    p.add_argument('--repeats',type=int,default=3)
    args = p.parse_args()
    if args.worlds < 1 or args.steps < 10 or args.steps % 10 or args.repeats < 1:
        p.error('Positive counts and steps divisible by ten required')
    run(args)
