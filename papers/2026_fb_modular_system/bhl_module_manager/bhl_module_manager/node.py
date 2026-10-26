"""ROS 2 module manager node (rclpy, tested logic lives in the pure-Python core).

Topics (std_msgs/String carries JSON so no custom interface package is needed)
  sub  modules/request   {"op": "attach", "port": "r_conn.hand", "module": "gripper_placeholder", "id": "r_grip", "host": "control_pc"}
                         {"op": "detach", "id": "r_hand"}
                         {"op": "load_config", "path": "/abs/path/config.yaml"}
                         {"op": "reload_descriptors"}
  pub  modules/response  result of each request: ok, reasons, events, capabilities added/removed
  pub  modules/state     latched: instances, free ports, joints, TF frames
  pub  capabilities      latched: available skills, unavailable skills with reasons, plan for "빼빼로 건네줘"
  pub  joint_states      zero values of the attach__* joints (attach_mode = dynamic), publish_rate Hz
On every change the composed URDF is pushed to robot_state_publisher's robot_description parameter.
"""
import json
import os

import rclpy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from .assembly import Result
from .capability import diff, evaluate, plan_hand_over
from .config import build, default_paths
from .descriptor import DescriptorError, load_hosts, load_library
from .urdf_gen import attach_joint_names, to_urdf


class ModuleManager(Node):
    def __init__(self):
        super().__init__("module_manager")
        mdir, hpath, cdir = default_paths()
        self.declare_parameter("module_dirs", [mdir])
        self.declare_parameter("hosts_file", hpath)
        self.declare_parameter("initial_config", os.path.join(cdir, "A_full.yaml"))
        self.declare_parameter("attach_mode", "dynamic")
        self.declare_parameter("rsp_node", "robot_state_publisher")
        self.declare_parameter("bhl_urdf", "")
        self.declare_parameter("mesh_prefix", "")
        self.declare_parameter("publish_rate", 20.0)
        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self.module_dirs = list(p("module_dirs"))
        self.hosts_file = p("hosts_file")
        self.attach_mode = p("attach_mode")
        self.bhl_urdf = p("bhl_urdf") or None
        self.mesh_prefix = p("mesh_prefix") or None

        latched = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_state = self.create_publisher(String, "modules/state", latched)
        self.pub_caps = self.create_publisher(String, "capabilities", latched)
        self.pub_resp = self.create_publisher(String, "modules/response", 10)
        self.pub_js = self.create_publisher(JointState, "joint_states", 10)
        self.create_subscription(String, "modules/request", self.on_request, 10)
        self.rsp_client = self.create_client(SetParameters, f"/{p('rsp_node').strip('/')}/set_parameters")

        self.library = load_library(*self.module_dirs)
        self.hosts = load_hosts(self.hosts_file)
        self.asm, _ = build(p("initial_config"), self.library, self.hosts)
        self.caps = evaluate(self.asm)
        self.urdf = None
        self.pending = True
        self.create_timer(1.0 / float(p("publish_rate")), self.on_timer)
        self.publish_model()
        self.get_logger().info(f"modules: {[i for i in self.asm.order]}, plan: {plan_hand_over(self.caps)['call']}")

    # ----------------------------------------------------------------- requests
    def on_request(self, msg):
        try:
            req = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.respond({"ok": False, "reasons": [f"JSON 오류: {e}"], "request": msg.data})
            return
        op = req.get("op")
        try:
            if op == "attach":
                r = self.asm.attach(req["port"], req["module"], req.get("id"), req.get("side"), req.get("host"))
            elif op == "detach":
                r = self.asm.detach(req["id"])
            elif op == "load_config":
                self.asm, _ = build(req["path"], self.library, self.hosts)
                r = Result(True, [], [f"구성 불러옴: {req['path']}"])
            elif op == "reload_descriptors":
                self.library = load_library(*self.module_dirs)
                self.asm.library = self.library
                r = Result(True, [], [f"명세 {len(self.library)}개 다시 읽음"])
            else:
                r = Result(False, [f"알 수 없는 op: {op}"])
        except (KeyError, ValueError, DescriptorError) as e:
            r = Result(False, [f"{type(e).__name__}: {e}"])
        resp = {"ok": r.ok, "reasons": r.reasons, "events": r.events, "request": req}
        if r.ok:
            new = evaluate(self.asm)
            added, removed = diff(self.caps, new)
            self.caps = new
            resp.update({"capabilities_added": added, "capabilities_removed": removed,
                         "hand_over_plan": plan_hand_over(new)})
            self.publish_model()
        for line in (r.events if r.ok else r.reasons):
            (self.get_logger().info if r.ok else self.get_logger().warning)(line)
        self.respond(resp)

    def respond(self, obj):
        self.pub_resp.publish(String(data=json.dumps(obj, ensure_ascii=False)))

    # ------------------------------------------------------------------ outputs
    def publish_model(self):
        self.urdf = to_urdf(self.asm, attach_mode=self.attach_mode, bhl_urdf=self.bhl_urdf, mesh_prefix=self.mesh_prefix)
        self.pending = True
        self.try_push_urdf()
        state = self.asm.snapshot()
        state["frames"] = [{"frame": f, "parent": par, "kind": k} for f, par, k in self.asm.tree()]
        self.pub_state.publish(String(data=json.dumps(state, ensure_ascii=False)))
        caps = dict(self.caps)
        caps["hand_over_plan"] = plan_hand_over(self.caps)
        self.pub_caps.publish(String(data=json.dumps(caps, ensure_ascii=False)))

    def try_push_urdf(self):
        if not self.pending or not self.rsp_client.service_is_ready():
            return
        req = SetParameters.Request()
        req.parameters = [Parameter(name="robot_description",
                                    value=ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=self.urdf))]
        self.pending = False
        fut = self.rsp_client.call_async(req)
        fut.add_done_callback(self.on_pushed)

    def on_pushed(self, fut):
        try:
            res = fut.result()
            ok = all(r.successful for r in res.results)
        except Exception as e:  # noqa: BLE001
            ok, res = False, e
        if not ok:
            self.pending = True
            self.get_logger().warning(f"robot_description 갱신 실패, 다시 시도: {res}")

    def on_timer(self):
        self.try_push_urdf()
        if self.attach_mode != "dynamic":
            return
        names = attach_joint_names(self.asm)
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = names
        js.position = [0.0] * len(names)
        self.pub_js.publish(js)


def main(args=None):
    rclpy.init(args=args)
    node = ModuleManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
