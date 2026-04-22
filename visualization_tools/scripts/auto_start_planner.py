#!/usr/bin/env python3

import rospy
from std_srvs.srv import Trigger
from planner_msgs.srv import pci_initialization


def normalize_ns(ns):
    ns = ns.strip()
    if not ns:
        return ""
    if not ns.startswith("/"):
        ns = "/" + ns
    return ns.rstrip("/")


def call_init(ns, timeout):
    service = ns + "/pci_initialization_trigger"
    rospy.loginfo("[auto_start] waiting for init service: %s", service)
    rospy.wait_for_service(service, timeout=timeout)
    client = rospy.ServiceProxy(service, pci_initialization)
    req = pci_initialization._request_class()
    res = client(req)
    return bool(res.success)


def call_start(ns, timeout):
    service = ns + "/planner_control_interface/std_srvs/automatic_planning"
    rospy.loginfo("[auto_start] waiting for start service: %s", service)
    rospy.wait_for_service(service, timeout=timeout)
    client = rospy.ServiceProxy(service, Trigger)
    res = client()
    return bool(res.success)


def main():
    rospy.init_node("auto_start_planner", anonymous=False)

    enable = rospy.get_param("~enable", True)
    if not enable:
        rospy.loginfo("[auto_start] disabled by param, exiting.")
        return

    robots = rospy.get_param("~robot_namespaces", ["R1", "R2", "R3"])
    startup_delay = float(rospy.get_param("~startup_delay_sec", 8.0))
    between_calls = float(rospy.get_param("~between_calls_sec", 0.5))
    service_timeout = float(rospy.get_param("~service_wait_timeout_sec", 120.0))
    init_retries = int(rospy.get_param("~init_retries", 3))
    start_retries = int(rospy.get_param("~start_retries", 3))

    normalized = [normalize_ns(x) for x in robots if normalize_ns(x)]
    if not normalized:
        rospy.logwarn("[auto_start] no valid robot namespaces, exiting.")
        return

    rospy.loginfo(
        "[auto_start] waiting %.1fs before triggering init/start for %s",
        startup_delay,
        ",".join(normalized),
    )
    rospy.sleep(startup_delay)

    # Step 1: initialize all robots.
    for ns in normalized:
        ok = False
        for attempt in range(1, init_retries + 1):
            try:
                ok = call_init(ns, service_timeout)
                if ok:
                    rospy.loginfo("[auto_start] init ok for %s", ns)
                    break
                rospy.logwarn("[auto_start] init returned false for %s", ns)
            except Exception as exc:
                rospy.logwarn(
                    "[auto_start] init failed for %s (attempt %d/%d): %s",
                    ns,
                    attempt,
                    init_retries,
                    str(exc),
                )
            rospy.sleep(between_calls)
        if not ok:
            rospy.logerr("[auto_start] init failed for %s after retries", ns)

    rospy.sleep(between_calls)

    # Step 2: start automatic planning for all robots.
    for ns in normalized:
        ok = False
        for attempt in range(1, start_retries + 1):
            try:
                ok = call_start(ns, service_timeout)
                if ok:
                    rospy.loginfo("[auto_start] automatic planning started for %s", ns)
                    break
                rospy.logwarn("[auto_start] start returned false for %s", ns)
            except Exception as exc:
                rospy.logwarn(
                    "[auto_start] start failed for %s (attempt %d/%d): %s",
                    ns,
                    attempt,
                    start_retries,
                    str(exc),
                )
            rospy.sleep(between_calls)
        if not ok:
            rospy.logerr("[auto_start] start failed for %s after retries", ns)

    rospy.loginfo("[auto_start] done.")


if __name__ == "__main__":
    main()
