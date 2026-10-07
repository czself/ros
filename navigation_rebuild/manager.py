"""Sequential navigation -> inspection -> next-goal coordinator."""

import json
import os
import time

import rospy
from std_msgs.msg import String

from .executor import NavExecutor
from .goals import load_goals
from .tasks import TaskHandler


class NavigationManager:
    def __init__(self):
        rospy.init_node("navigation_rebuild_manager")
        route_file = rospy.get_param("~route_file", "/root/navigation_rebuild/goals.json")
        self.goals = load_goals(route_file)
        self.output_dir = rospy.get_param("~output_dir", "/root/ros1_ws/navigation_rebuild_runs")
        self.stop_after = rospy.get_param("~stop_after", "")
        self.run_tasks = bool(rospy.get_param("~run_tasks", True))
        self.status = rospy.Publisher("/navigation_rebuild/status", String, queue_size=1, latch=True)
        self.executor = NavExecutor()
        self.tasks = TaskHandler(self.output_dir)
        self.events = []

    def publish(self, value):
        self.status.publish(String(value))
        rospy.loginfo("navigation rebuild: %s", value)

    def finish(self, ok, reason):
        self.executor.cancel()
        os.makedirs(self.output_dir, exist_ok=True)
        summary = {"result": "COMPLETE" if ok else "FAILED", "reason": reason,
                   "route": [goal.name for goal in self.goals], "events": self.events,
                   "finished_wall_time": time.time()}
        path = os.path.join(self.output_dir, "run_summary.json")
        with open(path, "w", encoding="utf-8") as stream:
            json.dump(summary, stream, ensure_ascii=False, indent=2)
        self.publish(("COMPLETE:" if ok else "FAILED:") + reason)
        return ok

    def run(self):
        for goal in self.goals:
            if rospy.is_shutdown():
                return self.finish(False, "SHUTDOWN")
            self.publish("GO:" + goal.name)
            result = self.executor.navigate(goal)
            self.events.append({"goal": goal.name, "phase": "navigation", **result.__dict__})
            if not result.ok:
                return self.finish(False, goal.name + ":" + result.reason)
            if self.run_tasks:
                for task in goal.tasks:
                    self.publish("TASK:" + goal.name + ":" + task)
                    ok, detail = self.tasks.run(task, goal)
                    self.events.append({"goal": goal.name, "phase": task, "ok": ok, "detail": detail})
                    if not ok:
                        return self.finish(False, goal.name + ":" + detail)
            if goal.name == self.stop_after:
                return self.finish(True, "STOP_AFTER_" + goal.name)
        return self.finish(True, "HOME_VERIFIED")


if __name__ == "__main__":
    manager = NavigationManager()
    raise SystemExit(0 if manager.run() else 1)
