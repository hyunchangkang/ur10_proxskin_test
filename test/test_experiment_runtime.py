"""Run with the installed package sourced. Uses an isolated fake robot, never UR10_right."""
import csv
from collections import deque
import errno
import fcntl
import json
import math
import os
from pathlib import Path
import pty
import re
import signal
import subprocess
import tempfile
import termios
import time

os.environ['ROS_DOMAIN_ID'] = '87'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
import numpy as np
import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


def fk(q):
    result = np.eye(4)
    for th, d, a, alpha in zip(q, [.1273,0,0,.163941,.1157,.0922],
                              [0,-.612,-.5723,0,0,0], [math.pi/2,0,0,math.pi/2,-math.pi/2,0]):
        c,s,ca,sa = math.cos(th),math.sin(th),math.cos(alpha),math.sin(alpha)
        result = result @ np.array([[c,-s*ca,s*sa,a*c], [s,c*ca,-c*sa,a*s],
                                    [0,sa,ca,d], [0,0,0,1]])
    return result

rclpy.init()
node = rclpy.create_node('csv_runtime_fake_feedback')
q = [4.585102277825143, -1.2739189658964618, -1.9028513060378087,
     -1.5871765381732077, 1.5092238144257408, -1.4331193555719475]
follow = True
commands = []
pending = deque()
freeze_below_z = None
publisher = node.create_publisher(JointState, '/csv_runtime_mock/joint_states', 10)

def publish():
    while pending and pending[0][0] <= time.monotonic():
        _, target = pending.popleft()
        q[:] = target
    msg = JointState()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.position = q.copy()
    msg.velocity = [0.0]*6
    publisher.publish(msg)

def command(msg):
    commands.append(list(msg.data))
    if follow and (freeze_below_z is None or fk(msg.data)[2,3] > freeze_below_z):
        pending.append((time.monotonic() + 0.04, list(msg.data)))

subscription = node.create_subscription(Float64MultiArray, '/csv_runtime_mock/targetJ', command, 10)
timer = node.create_timer(.004, publish)
process = None
master = None
output = ''

def drain():
    global output
    while True:
        try:
            part = os.read(master, 65536)
            if not part:
                return
            output += part.decode(errors='replace')
        except BlockingIOError:
            return
        except OSError as exc:
            if exc.errno == errno.EIO:
                return
            raise

def spin(seconds):
    end = time.monotonic()+seconds
    while time.monotonic()<end:
        rclpy.spin_once(node, timeout_sec=.004)
        drain()

def wait_for(check):
    end = time.monotonic()+65
    while time.monotonic()<end:
        spin(.01)
        if check():
            return
        if process.poll() is not None:
            break
    raise AssertionError(output)

def spawn(args):
    global process, master, output
    output = ''
    master, slave = pty.openpty()
    def terminal():
        os.setsid()
        fcntl.ioctl(slave, termios.TIOCSCTTY, 0)
    process = subprocess.Popen(args, stdin=slave, stdout=slave, stderr=slave, preexec_fn=terminal)
    os.close(slave)
    os.set_blocking(master, False)

def stop():
    if process is not None and process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        spin(.4)
        process.wait(timeout=8)
    if master is not None:
        os.close(master)

def inspect_csv(path, travel, completed=True):
    with path.open(newline='') as file:
        rows = list(csv.reader(file))
    assert rows[0] == ['time_s','robot_distance_mm','sensor_value'], rows[0]
    samples = rows[1:]
    assert len(samples)>10, len(samples)
    assert all(len(row)==3 and row[2]=='' for row in samples), samples[:5]
    assert all(re.fullmatch(r'-?\d+\.\d{2}', row[1]) for row in samples), samples[:5]
    timestamps = [float(row[0]) for row in samples]
    assert timestamps[0] == 0.0
    assert all(a<b for a,b in zip(timestamps,timestamps[1:])), 'duplicate or backwards feedback times'
    distances = [float(row[1]) for row in samples]
    assert abs(distances[0]-travel)<.2, distances[:5]
    if completed:
        assert abs(distances[-1]-travel)<.2, distances[-5:]
    print('CSV:',path.name,'rows:',len(samples),'distance range mm:',min(distances),max(distances),flush=True)
    return distances

try:
    with tempfile.TemporaryDirectory(prefix='ur10_setup_runtime_') as directory:
        root=Path(directory)
        config=root/'params.yaml'
        config.write_text('/**:\n  ros__parameters:\n    robot_name: csv_runtime_mock\n    enable_ecan: false\n'
                          '    scan_speed_mm_s: 1000.0\n    csv_save_dir: '+str(root/'data')+'\n')
        start_file=root/'start.json'
        common=['ros2','launch','ur10_proxskin_test','proxskin_soc_test.launch.py',
                'config_file:='+str(config),'start_file:='+str(start_file)]
        spawn(common+['x_span_mm:=2','spacing_mm:=1'])
        wait_for(lambda: 'Choose mode' in output)
        base_j1=q[0]
        base_j6=q[5]
        os.write(master,b'0\n')
        wait_for(lambda: output.count('UR10 Motion Test')>=2)
        assert not start_file.exists(), 'base must not save a start'
        expected=[-math.pi/2,-math.pi/2,-math.pi/2,math.pi/2,base_j6]
        assert abs(q[0]-base_j1)<1e-6
        assert max(abs(a-b) for a,b in zip(q[1:],expected))<.004
        assert all(abs(command[5]-expected[4])<1e-12 for command in commands)
        print('PASS: mode 0 moves to base, holds J1/J6 and returns to menu',flush=True)
        os.write(master,b'1\n')
        wait_for(lambda: 'Use current position as start?' in output)
        spin(.2)
        count=len(commands)
        spin(.25)
        assert len(commands)==count, 'start question must not stream targets'
        os.write(master,b'n\n')
        wait_for(lambda: 'Z adjustment:' in output)
        count=len(commands)
        # Simulate an external pendant movement while terminal control is idle.
        pending.clear()
        q[0]+=.01
        pendant_q=q.copy()
        spin(.25)
        assert q==pendant_q and len(commands)==count, 'program must not overwrite pendant movement'
        anchor=fk(q)
        os.write(master,b'+5\n')
        wait_for(lambda: 'Calibration Z adjustment reached' in output)
        spin(.2)
        adjusted=fk(q)
        assert abs(adjusted[2,3]-anchor[2,3]-.005)<.0005
        assert np.linalg.norm(adjusted[:2,3]-anchor[:2,3])<.0005
        assert np.linalg.norm(adjusted[:3,:3]-anchor[:3,:3])<.002
        count=len(commands)
        spin(.2)
        assert len(commands)==count, 'adjustment prompt must stop streaming targets'
        alignment_j6=q[5]
        os.write(master,b'e\n')
        wait_for(lambda: output.count('Use current position as start?')>=2)
        os.write(master,b'y\n100\n')
        wait_for(lambda: output.count('UR10 Motion Test')>=3)
        assert 'Aligned experiment start saved' in output, output
        saved=json.loads(start_file.read_text())
        assert saved['version']==2 and saved['start_distance_mm']==100
        assert saved['orientation_aligned'] is True
        assert abs(saved['joint_rad'][5]-alignment_j6)<.002
        assert np.linalg.norm(fk(saved['joint_rad'])[:3,3]-adjusted[:3,3])<.0001
        assert not (root/'data').exists(), 'start setup must not create experiment CSV'
        print('PASS: mode 1 n/pendant/+5/e/y/100 aligns at fixed XYZ and saves measured reference',flush=True)
        for mode, inputs, prefix, menu_count in [
            ('2',b'2\n1\n','vertical',4),
            ('3',b'3\n1\n','columns',5),
            ('4',b'4\n1\n0.03\n','stepped',6),
        ]:
            begin=len(output)
            os.write(master,inputs)
            wait_for(lambda: output.count('UR10 Motion Test')>=menu_count)
            part=output[begin:]
            assert 'Endpoint reached' in part, part
            assert 'Endpoint not reached' not in part and 'SAFETY ABORT' not in part and 'No IK solution' not in part, part
            files=list((root/'data').glob('experiment_'+prefix+'_*.csv'))
            assert len(files)==1,files
            distances=inspect_csv(files[0],100)
            assert abs(min(distances))<.11,min(distances)
            with files[0].open(newline='') as file:
                rows=list(csv.DictReader(file))
            bottom=[float(row['time_s']) for row in rows if abs(float(row['robot_distance_mm']))<.11]
            assert bottom[-1]-bottom[0]>=.2, 'bottom must have settled feedback before return'
            print('PASS: mode',mode,'waits for actual endpoints, uses saved 100mm and returns to menu',flush=True)
        os.write(master,b'1\ny\n300\n')
        wait_for(lambda: output.count('UR10 Motion Test')>=7)
        saved=json.loads(start_file.read_text())
        assert saved['start_distance_mm']==300
        os.write(master,b'2\n1\n')
        wait_for(lambda: output.count('UR10 Motion Test')>=8)
        files=list((root/'data').glob('experiment_vertical_*.csv'))
        assert len(files)==2,files
        latest=max(files,key=lambda p:p.stat().st_mtime_ns)
        distances=inspect_csv(latest,300)
        assert abs(min(distances))<.11,min(distances)
        print('PASS: resaving 300mm changes both motion and CSV reference',flush=True)
        original=start_file.read_bytes()
        os.write(master,b'1\nq\n')
        wait_for(lambda: output.count('UR10 Motion Test')>=9)
        assert start_file.read_bytes()==original
        print('PASS: cancelling start setup preserves saved reference',flush=True)
        log=output
        stop()
        pending.clear()
        q[:]=saved['joint_rad']
        freeze_below_z=saved['tcp_pose'][2]-.3+.0025
        config.write_text(config.read_text()+'    endpoint_timeout_seconds: 0.8\n')
        spawn(common+['mode:=vertical','repeat_count:=1'])
        wait_for(lambda: process.poll() is not None)
        assert 'Endpoint not reached' in output,output
        assert 'Vertical scan completed' not in output,output
        assert 'CSV logging finished' in output,output
        latest=max((root/'data').glob('experiment_vertical_*.csv'),key=lambda p:p.stat().st_mtime_ns)
        distances=inspect_csv(latest,300,completed=False)
        assert min(distances)>2, min(distances)
        assert max(distances)-min(distances)>290
        print('PASS: stops at an unreached bottom, retains measured CSV and never reports completion',flush=True)
        Path('/tmp/ur10_setup_runtime_result.log').write_text(log+'\n'+output)
        stop()
finally:
    if process is not None and process.poll() is None:
        stop()
    node.destroy_node()
    rclpy.shutdown()
