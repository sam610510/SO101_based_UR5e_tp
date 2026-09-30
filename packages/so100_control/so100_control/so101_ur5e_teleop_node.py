#!/usr/bin/env python3
import sys, os
current_dir = os.path.dirname(os.path.abspath(__file__))
if current_dir not in sys.path: sys.path.insert(0, current_dir)
parent_dir = os.path.dirname(current_dir)
if parent_dir not in sys.path: sys.path.insert(0, parent_dir)

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration as MsgDuration
import numpy as np
import xml.etree.ElementTree as ET
import scservo_sdk as scs
import socket
import threading
import time

def rot_x(a): return np.array([[1,0,0,0],[0,np.cos(a),-np.sin(a),0],[0,np.sin(a),np.cos(a),0],[0,0,0,1]])
def rot_y(a): return np.array([[np.cos(a),0,np.sin(a),0],[0,1,0,0],[-np.sin(a),0,np.cos(a),0],[0,0,0,1]])
def rot_z(a): return np.array([[np.cos(a),-np.sin(a),0,0],[np.sin(a),np.cos(a),0,0],[0,0,1,0],[0,0,0,1]])
def trans(x,y,z):
    T=np.eye(4); T[0,3]=x; T[1,3]=y; T[2,3]=z; return T
def rpy_mat(r,p,y): return rot_z(y)@rot_y(p)@rot_x(r)
def joint_tf(xyz,rpy,q,axis):
    T=trans(*xyz)@rpy_mat(*rpy)
    a=np.array(axis); c,s=np.cos(q),np.sin(q); v=1-c; x,y,z=a
    R=np.array([[x*x*v+c,x*y*v-z*s,x*z*v+y*s],
                [x*y*v+z*s,y*y*v+c,y*z*v-x*s],
                [x*z*v-y*s,y*z*v+x*s,z*z*v+c]])
    Tr=np.eye(4); Tr[:3,:3]=R
    return T@Tr

SO101_JOINTS = [
    ([0.0388353,0,0.0624],   [np.pi,0,-np.pi],       [0,0,1]),
    ([-0.0303992,-0.0182778,-0.0542],[-np.pi/2,-np.pi/2,0],[0,0,1]),
    ([-0.11257,-0.028,0],    [0,0,np.pi/2],          [0,0,1]),
    ([-0.1349,0.0052,0],     [0,0,-np.pi/2],         [0,0,1]),
    ([0,-0.0611,0.0181],     [np.pi/2,0.0487,np.pi], [0,0,1]),
    ([0.0202,0.0188,-0.0234],[np.pi/2,0,0],          [0,0,1]),
]
GRIPPER_FRAME_TF = ([-0.0079,-0.000218121,-0.0981274],[0,np.pi,0])

def _mat_to_quat(R):
    import numpy as np
    t = np.trace(R)
    if t > 0:
        s = 0.5 / np.sqrt(t + 1.0)
        w = 0.25 / s
        x = (R[2,1]-R[1,2])*s; y = (R[0,2]-R[2,0])*s; z = (R[1,0]-R[0,1])*s
    else:
        i = np.argmax([R[0,0],R[1,1],R[2,2]])
        if i==0:
            s = 2.0*np.sqrt(1.0+R[0,0]-R[1,1]-R[2,2])
            w=(R[2,1]-R[1,2])/s; x=0.25*s; y=(R[0,1]+R[1,0])/s; z=(R[0,2]+R[2,0])/s
        elif i==1:
            s = 2.0*np.sqrt(1.0+R[1,1]-R[0,0]-R[2,2])
            w=(R[0,2]-R[2,0])/s; x=(R[0,1]+R[1,0])/s; y=0.25*s; z=(R[1,2]+R[2,1])/s
        else:
            s = 2.0*np.sqrt(1.0+R[2,2]-R[0,0]-R[1,1])
            w=(R[1,0]-R[0,1])/s; x=(R[0,2]+R[2,0])/s; y=(R[1,2]+R[2,1])/s; z=0.25*s
    q = np.array([x,y,z,w]); return q/np.linalg.norm(q)

def _quat_to_mat(q):
    import numpy as np
    x,y,z,w = q/np.linalg.norm(q)
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
        [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)]])

def _slerp(q0, q1, alpha):
    import numpy as np
    q0 = q0/np.linalg.norm(q0); q1 = q1/np.linalg.norm(q1)
    dot = np.dot(q0, q1)
    if dot < 0:
        q1 = -q1; dot = -dot
    if dot > 0.9995:
        r = q0 + alpha*(q1-q0); return r/np.linalg.norm(r)
    theta = np.arccos(np.clip(dot,-1,1))
    s0 = np.sin((1-alpha)*theta)/np.sin(theta)
    s1 = np.sin(alpha*theta)/np.sin(theta)
    return s0*q0 + s1*q1

def so101_fk_full(q6):
    T=np.eye(4)
    for i in range(5):
        xyz,rpy,axis = SO101_JOINTS[i]
        T=T@joint_tf(xyz,rpy,q6[i],axis)
    T=T@joint_tf(GRIPPER_FRAME_TF[0],GRIPPER_FRAME_TF[1],0.0,[0,0,1])
    return T

def _H(r,p,y,x,yy,z):
    cr,sr=np.cos(r),np.sin(r); cp,sp=np.cos(p),np.sin(p); cy,sy=np.cos(y),np.sin(y)
    R=np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]])@np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]])@np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]])
    T=np.eye(4); T[:3,:3]=R; T[:3,3]=[x,yy,z]; return T
def _Rz(q):
    c,s=np.cos(q),np.sin(q); T=np.eye(4); T[:3,:3]=[[c,-s,0],[s,c,0],[0,0,1]]; return T

UR5E_JOINT_NAMES = [
    'shoulder_pan_joint','shoulder_lift_joint','elbow_joint',
    'wrist_1_joint','wrist_2_joint','wrist_3_joint',
]

UR5E_LIMITS = [(-2*np.pi,2*np.pi),(-2*np.pi,2*np.pi),
               (0.1, 8/9*np.pi),
               (-2*np.pi,2*np.pi),(-2*np.pi,2*np.pi),(-2*np.pi,2*np.pi)]

SO101_GRIP_OPEN_RAW  = 2400
SO101_GRIP_CLOSE_RAW = 1536
GRIP_MIN_CHANGE = 2
GRIP_SEND_PERIOD = 0.04

def map_gripper_pos(raw):
    span = SO101_GRIP_CLOSE_RAW - SO101_GRIP_OPEN_RAW
    if abs(span) < 1e-6:
        return 0
    t = (raw - SO101_GRIP_OPEN_RAW) / span
    t = max(0.0, min(1.0, t))
    return int(round(t * 255))


class GripperLink:

    def __init__(self, ip, port=63352, timeout=0.3, logger=None, send_hz=25.0):
        self.ip = ip
        self.port = port
        self.timeout = timeout
        self.logger = logger
        self.sock = None
        self._target = None
        self._last_sent = None
        self._lock = threading.Lock()
        self._running = True
        self._send_period = 1.0/send_hz
        self._connect()

        self._thread = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()

    def _log(self, msg):
        if self.logger:
            self.logger.info(msg)

    def _connect(self):
        try:
            if self.sock is not None:
                self.sock.close()
        except Exception:
            pass
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        s.settimeout(self.timeout)
        s.connect((self.ip, self.port))
        self.sock = s
        self._log(f"Gripper socket connected {self.ip}:{self.port} (TCP_NODELAY, background thread)")

    def set_target(self, pos):
        pos = max(0, min(255, int(pos)))
        with self._lock:
            self._target = pos

    def _worker(self):
        while self._running:
            t0 = time.time()
            with self._lock:
                tgt = self._target
            if tgt is not None and tgt != self._last_sent:
                try:
                    self.sock.sendall(f"SET POS {tgt}\n".encode())
                    self.sock.recv(64)
                    self._last_sent = tgt
                except (socket.timeout, OSError):
                    try:
                        self._connect()
                    except Exception:
                        pass

            dt = time.time() - t0
            if dt < self._send_period:
                time.sleep(self._send_period - dt)

    def close(self):
        self._running = False
        try:
            if self._thread.is_alive():
                self._thread.join(timeout=1.0)
        except Exception:
            pass
        try:
            if self.sock is not None:
                self.sock.close()
        except Exception:
            pass

UR5E_TCP_LEN = 0.25

def ur5e_fk_full(q):
    T=np.eye(4)
    T=T@_H(0,0,np.pi,0,0,0)
    T=T@_H(0,0,0,0,0,0.1625)@_Rz(q[0])
    T=T@_H(1.570796327,0,0,0,0,0)@_Rz(q[1])
    T=T@_H(0,0,0,-0.425,0,0)@_Rz(q[2])
    T=T@_H(0,0,0,-0.3922,0,0.1333)@_Rz(q[3])
    T=T@_H(1.570796327,0,0,0,-0.0997,0)@_Rz(q[4])
    T=T@_H(1.570796326589793,np.pi,np.pi,0,0.0996,0)@_Rz(q[5])
    T=T@_H(0,-1.5707963267948966,-1.5707963267948966,0,0,0)
    T=T@_H(1.5707963267948966,0,1.5707963267948966,0,0,0)
    T_tcp=np.eye(4); T_tcp[2,3]=UR5E_TCP_LEN
    T=T@T_tcp
    return T
def ur5e_fk_pos(q): return ur5e_fk_full(q)[:3,3]

SO101_WS = {'x':(-0.07,0.27),'y':(-0.23,0.23),'z':(0.0,0.30)}
UR5E_WS  = {'x':(-0.55,0.55),'y':(-0.55,0.55),'z':(0.05,0.60)}
def ws_map(pos):
    so=list(SO101_WS.values()); ur=list(UR5E_WS.values())
    n=np.clip([(pos[i]-so[i][0])/(so[i][1]-so[i][0]) for i in range(3)],0,1)
    return np.array([n[i]*(ur[i][1]-ur[i][0])+ur[i][0] for i in range(3)])

def ur5e_jacobian_full(q, delta=1e-5):
    J=np.zeros((6,6)); T0=ur5e_fk_full(q); p0=T0[:3,3]
    for i in range(6):
        dq=q.copy(); dq[i]+=delta; T1=ur5e_fk_full(dq)
        J[:3,i]=(T1[:3,3]-p0)/delta; dR=T1[:3,:3]@T0[:3,:3].T
        J[3,i]=(dR[2,1]-dR[1,2])/(2*delta)
        J[4,i]=(dR[0,2]-dR[2,0])/(2*delta)
        J[5,i]=(dR[1,0]-dR[0,1])/(2*delta)
    return J

def ur5e_ik_realtime(target_T, q_warm, max_iter=40, tol=1e-3, step=0.5):
    tp=target_T[:3,3]; tR=target_T[:3,:3]; q=q_warm.copy()
    for _ in range(max_iter):
        T=ur5e_fk_full(q); ep=tp-T[:3,3]; dR=tR@T[:3,:3].T
        er=np.array([(dR[2,1]-dR[1,2])/2,(dR[0,2]-dR[2,0])/2,(dR[1,0]-dR[0,1])/2])
        if np.linalg.norm(ep)<tol and np.linalg.norm(er)<0.05:
            return q, True
        J=ur5e_jacobian_full(q); lam=0.1
        J_pinv=J.T@np.linalg.inv(J@J.T+lam**2*np.eye(6))
        dq=np.clip(step*J_pinv@np.concatenate([ep,er*0.5]),-0.15,0.15); q=q+dq
        for i,(lo,hi) in enumerate(UR5E_LIMITS): q[i]=np.clip(q[i],lo,hi)
    perr=np.linalg.norm(tp-ur5e_fk_full(q)[:3,3])
    return q, (perr<0.01)

UR5E_ELBOW_UP_SEEDS = [
    np.array([-0.0612, -2.2007, 2.0521, -1.5937, 4.5341, 0.7596]),
    np.array([-0.0612, -1.9000, 1.9000, -1.5000, 4.5341, 0.7596]),
    np.array([ 0.3000, -2.2007, 2.0521, -1.5937, 4.5341, 0.7596]),
    np.array([-0.4000, -2.2007, 2.0521, -1.5937, 4.5341, 0.7596]),
]
def ur5e_ik_full(target_T, q_warm=None, max_iter=150, tol=1e-3, step=0.5, jump_margin=0.02):
    seeds = list(UR5E_ELBOW_UP_SEEDS)
    if q_warm is not None: seeds=[q_warm.copy()]+seeds
    tp=target_T[:3,3]; tR=target_T[:3,:3]
    candidates=[]
    for seed in seeds:
        q=seed.copy()
        for _ in range(max_iter):
            T=ur5e_fk_full(q); ep=tp-T[:3,3]; dR=tR@T[:3,:3].T
            er=np.array([(dR[2,1]-dR[1,2])/2,(dR[0,2]-dR[2,0])/2,(dR[1,0]-dR[0,1])/2])
            if np.linalg.norm(ep)<tol and np.linalg.norm(er)<0.05: break
            J=ur5e_jacobian_full(q); lam=0.1
            J_pinv=J.T@np.linalg.inv(J@J.T+lam**2*np.eye(6))
            dq=np.clip(step*J_pinv@np.concatenate([ep,er*0.5]),-0.1,0.1); q=q+dq
            for i,(lo,hi) in enumerate(UR5E_LIMITS): q[i]=np.clip(q[i],lo,hi)
        Tf=ur5e_fk_full(q); ep_f=np.linalg.norm(tp-Tf[:3,3])
        candidates.append((ep_f,q.copy()))
    best_err=min(c[0] for c in candidates)
    close=[c for c in candidates if c[0]<=best_err+jump_margin]
    if q_warm is not None and len(close)>1:
        best_err,best_q=min(close,key=lambda c:np.linalg.norm(c[1]-q_warm))
    else:
        best_err,best_q=min(candidates,key=lambda c:c[0])
    return best_q,best_err

R_BASE_180 = np.array([[-1., 0., 0.],
                       [ 0.,-1., 0.],
                       [ 0., 0., 1.]])

UR5E_HOME_Q = np.array([-0.2401, -1.8595, 1.6841, -1.4915, 4.6957, 1.1680])
SO101_ARM_REF = np.array([0.2288, 0.0122, 0.1364])
ARM_THRESH = 0.02

def fk_chain_points(q):
    T=np.eye(4)
    T=T@_H(0,0,np.pi,0,0,0)@_H(0,0,0,0,0,0.1625)@_Rz(q[0])
    T=T@_H(1.570796327,0,0,0,0,0)@_Rz(q[1])
    T=T@_H(0,0,0,-0.425,0,0)@_Rz(q[2])
    elbow=T[:3,3].copy()
    T=T@_H(0,0,0,-0.3922,0,0.1333)@_Rz(q[3])
    T=T@_H(1.570796327,0,0,0,-0.0997,0)@_Rz(q[4])
    T=T@_H(1.570796326589793,np.pi,np.pi,0,0.0996,0)@_Rz(q[5])
    T=T@_H(0,-1.5707963267948966,-1.5707963267948966,0,0,0)
    T=T@_H(1.5707963267948966,0,1.5707963267948966,0,0,0)
    flange=T[:3,3].copy()
    T_tcp=np.eye(4); T_tcp[2,3]=UR5E_TCP_LEN
    T=T@T_tcp
    return {'elbow':elbow, 'flange':flange, 'tool0':T[:3,3].copy()}

def is_folded_unsafe(q):
    pts = fk_chain_points(q)
    ee = pts['tool0']
    ee_h = np.hypot(ee[0], ee[1])
    if ee[2] < 0.25 and ee_h < 0.22:
        return True, f"end-effector low({ee[2]:.2f}) and close({ee_h:.2f})"
    if ee[2] < 0.02:
        return True, "end-effector too low"
    base_dist = np.linalg.norm(ee)
    if base_dist < 0.20:
        return True, f"end-effector too close to base({base_dist:.2f}m)"
    elbow = pts['elbow']
    elbow_dist = np.linalg.norm(elbow)
    if elbow_dist < 0.25:
        return True, f"elbow too close to base({elbow_dist:.2f}m)"
    flange = pts['flange']
    flange_dist = np.linalg.norm(flange)
    if flange_dist < 0.15:
        return True, f"flange too close to base({flange_dist:.2f}m)"
    return False, ""

class UR5eTeleopNode(Node):
    def __init__(self):
        super().__init__('ur5e_teleop_node')
        self.declare_parameter('leader_port','/dev/ttyACM0')
        self.declare_parameter('loop_hz',30.0)
        self.declare_parameter('max_jump',0.5)
        self.declare_parameter('filter_alpha', 0.7)

        l_port=self.get_parameter('leader_port').value
        hz=self.get_parameter('loop_hz').value
        self.max_jump=self.get_parameter('max_jump').value
        self.filter_alpha=self.get_parameter('filter_alpha').value

        self.get_logger().info("="*50)
        self.get_logger().info("  SO101 -> UR5e absolute teleoperation (safety version)")
        self.get_logger().info(f"  Leader serial port: {l_port}  |  rate: {hz} Hz")
        self.get_logger().info(f"  Starts at home, follows only once SO101 is in position")
        self.get_logger().info("="*50)

        self.calib=[2078, 2060, 1985, 2042, 2047, 2136]

        self.urdf_limits={n:(-2*np.pi,2*np.pi) for n in UR5E_JOINT_NAMES}
        self._usub=self.create_subscription(String,'/robot_description',self._urdf_cb,10)

        self.leader_ids=[1,2,3,4,5,6]
        self.port=scs.PortHandler(l_port)
        self.packet=scs.protocol_packet_handler(self.port,0)
        if not self.port.openPort() or not self.port.setBaudRate(1000000):
            self.get_logger().error(f"Failed to open {l_port}"); return
        for lid in self.leader_ids:
            self.packet.write1ByteTxRx(lid,19,2)
            self.packet.write1ByteTxRx(lid,40,0)
        self.get_logger().info("SO101 connected (torque released)")

        self._armed = False
        self._last_pos=np.array([0.10,0.0,0.15])
        self._last_q=UR5E_HOME_Q.copy()
        self._first=True
        self._fallback_count=0

        self.ACTION_NAME='/scaled_joint_trajectory_controller/follow_joint_trajectory'
        self._traj_pub=self.create_publisher(
            JointTrajectory,'/scaled_joint_trajectory_controller/joint_trajectory',10)
        self._traj_action=ActionClient(self,FollowJointTrajectory,self.ACTION_NAME)

        self.declare_parameter('robot_ip', 'None')
        robot_ip = self.get_parameter('robot_ip').value
        if robot_ip == 'None':
            self.get_logger().error(
                "Missing robot_ip parameter! Launch with --ros-args -p robot_ip:=<your robot IP>")
            raise RuntimeError("robot_ip not set")
        self._gripper = GripperLink(robot_ip, logger=self.get_logger())
        self._gripper_last_pos = None
        self._gripper_last_send_t = 0.0

        self.declare_parameter('raw_filter_alpha', 0.3)
        self.declare_parameter('pose_filter_alpha', 0.3)
        self.pose_alpha = self.get_parameter('pose_filter_alpha').value
        self._quat_filtered = None
        self.raw_alpha = self.get_parameter('raw_filter_alpha').value
        self._rads_filtered = None

        self.declare_parameter('max_dq_per_cmd', 0.25)
        self.max_dq=self.get_parameter('max_dq_per_cmd').value

        self.declare_parameter('armed_settle_sec', 1.5)
        self.declare_parameter('armed_settle_dq_scale', 0.2)
        self.armed_settle_sec=self.get_parameter('armed_settle_sec').value
        self.armed_settle_scale=self.get_parameter('armed_settle_dq_scale').value
        self._armed_time=None

        self.declare_parameter('max_joint_vel', 1.0)
        self.max_joint_vel=self.get_parameter('max_joint_vel').value
        self._prev_pub_q=None
        self._prev_pub_time=None

        self._homed=False
        self.declare_parameter('home_duration', 10.0)
        self.declare_parameter('max_home_deg_per_sec', 15.0)
        home_dur=self.get_parameter('home_duration').value
        self.get_logger().info(f"Startup: moving UR5e to home ({home_dur:.0f}s)...")
        self._goto_home_blocking(duration=home_dur)

        self._timer=self.create_timer(1.0/hz,self._loop)
        self.get_logger().info(f"UR5e is at home. Move SO101 near reference position {SO101_ARM_REF} to start following")

    def _urdf_cb(self,msg):
        try:
            root=ET.fromstring(msg.data)
            for j in root.findall('joint'):
                name=j.get('name')
                if name in self.urdf_limits:
                    lim=j.find('limit')
                    if lim is not None:
                        self.urdf_limits[name]=(float(lim.get('lower',-2*np.pi)),float(lim.get('upper',2*np.pi)))
            self.get_logger().info("UR5e URDF joint limits synced")
            self.destroy_subscription(self._usub)
        except Exception as e: self.get_logger().error(f"URDF parse failed: {e}")

    def _read_current_q(self, timeout=2.0):
        got={'q':None}
        def cb(msg):
            q=[None]*6
            for i,name in enumerate(UR5E_JOINT_NAMES):
                if name in msg.name: q[i]=msg.position[msg.name.index(name)]
            if all(v is not None for v in q): got['q']=np.array(q)
        sub=self.create_subscription(JointState,'/joint_states',cb,10)
        t0=self.get_clock().now()
        while got['q'] is None:
            rclpy.spin_once(self,timeout_sec=0.05)
            if (self.get_clock().now()-t0).nanoseconds>timeout*1e9: break
        self.destroy_subscription(sub)
        return got['q']

    def _goto_home_blocking(self, duration=10.0):
        cur=self._read_current_q()
        if cur is None:
            self.get_logger().error("Cannot read /joint_states, unable to move to home; check that the UR driver is running")
            return False
        move=np.degrees(np.max(np.abs(UR5E_HOME_Q-cur)))
        self.get_logger().info(
            f"  current(deg)=[{', '.join(f'{np.degrees(v):+.0f}' for v in cur)}]")
        self.get_logger().info(
            f"  home(deg)=[{', '.join(f'{np.degrees(v):+.0f}' for v in UR5E_HOME_Q)}]  max move {move:.0f} deg")

        max_deg_per_sec=self.get_parameter('max_home_deg_per_sec').value
        duration=max(duration, move/max(max_deg_per_sec,1e-3))
        self.get_logger().info(f"  duration at max {max_deg_per_sec:.0f} deg/s: {duration:.1f}s")

        if not self._traj_action.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f"Action server {self.ACTION_NAME} not found; check that the controller is active")
            return False
        goal=FollowJointTrajectory.Goal()
        tj=JointTrajectory(); tj.joint_names=UR5E_JOINT_NAMES
        pt=JointTrajectoryPoint(); pt.positions=UR5E_HOME_Q.tolist(); pt.velocities=[0.]*6
        pt.time_from_start=MsgDuration(sec=int(duration),nanosec=int((duration%1)*1e9))
        tj.points=[pt]; goal.trajectory=tj
        self.get_logger().info("Moving to home...")
        sf=self._traj_action.send_goal_async(goal)
        rclpy.spin_until_future_complete(self,sf)
        h=sf.result()
        if not h or not h.accepted:
            self.get_logger().error("Home goal rejected"); return False
        rf=h.get_result_async(); rclpy.spin_until_future_complete(self,rf)
        self._homed=True; self._last_q=UR5E_HOME_Q.copy()
        self.get_logger().info("Reached home")
        return True

    def _publish(self, q):
        q_cmd=q.copy()
        for i,name in enumerate(UR5E_JOINT_NAMES):
            lo,hi=self.urdf_limits[name]; q_cmd[i]=np.clip(q_cmd[i],lo,hi)
        dq_limit=self.max_dq
        if self._armed_time is not None:
            elapsed=time.time()-self._armed_time
            if elapsed<self.armed_settle_sec:
                ramp=self.armed_settle_scale+(1.0-self.armed_settle_scale)*(elapsed/self.armed_settle_sec)
                dq_limit=self.max_dq*ramp
        clamp_ref=self._prev_pub_q if self._prev_pub_q is not None else self._last_q
        dq=q_cmd-clamp_ref
        dq=np.clip(dq,-dq_limit,dq_limit)
        q_cmd=clamp_ref+dq

        now=time.time()
        if self._prev_pub_q is None:
            vel=np.zeros(6)
        else:
            dt_v=max(now-self._prev_pub_time,1e-3)
            vel=np.clip((q_cmd-self._prev_pub_q)/dt_v,-self.max_joint_vel,self.max_joint_vel)
        self._prev_pub_q=q_cmd.copy(); self._prev_pub_time=now

        tj=JointTrajectory(); tj.joint_names=UR5E_JOINT_NAMES
        pt=JointTrajectoryPoint(); pt.positions=q_cmd.tolist(); pt.velocities=vel.tolist()
        loop_period = 1.0/self.get_parameter('loop_hz').value
        dt=max(loop_period*2.0, 0.1)
        pt.time_from_start=MsgDuration(sec=int(dt),nanosec=int((dt%1)*1e9))
        tj.points=[pt]
        self._traj_pub.publish(tj)

    def _update_gripper(self, target_pos):
        self._gripper.set_target(target_pos)

    def _loop(self):
        rads=[0.]*6; raws=[0]*6; ok=[False]*6
        for i,lid in enumerate(self.leader_ids):
            raw,res,_=self.packet.read2ByteTxRx(lid,56)
            if res==scs.COMM_SUCCESS:
                raws[i]=raw
                rads[i]=(raw-self.calib[i])*(2.*math.pi/4096.); ok[i]=True
        if not all(ok):
            self.get_logger().warn("Some joints failed to read", throttle_duration_sec=2.0)
            return

        rads_arr = np.array(rads)
        if self._rads_filtered is None:
            self._rads_filtered = rads_arr.copy()
        else:
            a = self.raw_alpha
            self._rads_filtered[:5] = a*rads_arr[:5] + (1.0-a)*self._rads_filtered[:5]
            self._rads_filtered[5] = rads_arr[5]
        rads = self._rads_filtered

        self._update_gripper(map_gripper_pos(raws[5]))

        so101_T = so101_fk_full(rads)
        so101_pos = so101_T[:3,3]

        if not self._armed:
            dist = np.linalg.norm(so101_pos - SO101_ARM_REF)
            self.get_logger().info(
                f"Standby (robot at home)... SO101 distance to reference {dist:.3f}m (need < {ARM_THRESH})",
                throttle_duration_sec=1.0)
            if dist < ARM_THRESH:
                self._armed = True
                self._last_q = UR5E_HOME_Q.copy()
                self._last_pos = so101_pos.copy()
                self._first = False
                self._armed_time = time.time()
                self.get_logger().info("SO101 in position, starting to follow!")
            return

        jump=np.linalg.norm(so101_pos-self._last_pos)
        if jump>self.max_jump:
            self.get_logger().warn(f"Position jump {jump:.3f}m, skipping", throttle_duration_sec=0.5)
            return
        self._last_pos=so101_pos.copy()

        ur5e_pos = ws_map(so101_pos)
        target_T = np.eye(4)
        q_target = _mat_to_quat(so101_T[:3,:3])
        if self._quat_filtered is None:
            self._quat_filtered = q_target
        else:
            self._quat_filtered = _slerp(self._quat_filtered, q_target, self.pose_alpha)
        target_T[:3,:3] = _quat_to_mat(self._quat_filtered)
        target_T[:3,3]  = ur5e_pos

        q, ok_ik = ur5e_ik_realtime(target_T, self._last_q.copy())
        if not ok_ik:
            q, _ = ur5e_ik_full(target_T, q_warm=self._last_q.copy())
            self._fallback_count += 1

        unsafe, reason = is_folded_unsafe(q)
        if unsafe:
            self.get_logger().warn(
                f"{reason}, freezing pose (move SO101 back to the safe zone to recover)",
                throttle_duration_sec=0.5)
            self._publish(self._last_q)
            return

        ee = ur5e_fk_pos(q)

        pos_err = np.linalg.norm(ur5e_pos - ee)
        if pos_err < 0.05:
            alpha = self.filter_alpha
            blended_q = alpha * q + (1.0 - alpha) * self._last_q
            blended_unsafe, blended_reason = is_folded_unsafe(blended_q)
            if blended_unsafe:
                self.get_logger().warn(
                    f"After blending: {blended_reason}, discarding this update (keeping last safe pose)",
                    throttle_duration_sec=0.5)
            else:
                self._last_q = blended_q

        self._publish(self._last_q)

        q_str=' '.join(f'{v:+.2f}' for v in self._last_q)
        tilt=np.degrees(np.arccos(np.clip(-ur5e_fk_full(self._last_q)[2,2],-1.0,1.0)))
        self.get_logger().info(
            f"UR5e({ur5e_pos[0]:+.3f},{ur5e_pos[1]:+.3f},{ur5e_pos[2]:+.3f})"
            f" tilt={tilt:.1f}deg posErr={pos_err*1000:.1f}mm fb={self._fallback_count} q=[{q_str}]",
            throttle_duration_sec=1.0)

def main(args=None):
    rclpy.init(args=args)
    node=UR5eTeleopNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        if hasattr(node,'port') and node.port: node.port.closePort()
        if hasattr(node,'_gripper') and node._gripper: node._gripper.close()
        node.destroy_node(); rclpy.shutdown()

if __name__=='__main__': main()
