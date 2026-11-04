#!/usr/bin/env python3
"""Drive the real page in a real browser against the simulated controller.

    uv run rac up --sim                      # in one terminal
    uv run python tests/e2e/run_web_e2e.py   # in another

Uses the Edge or Chrome already on the machine through Playwright, walks an operator
through every tab, checks that what the page shows comes from controller feedback,
and saves a screenshot per step. Exits non-zero on the first failed check, and fails
on any uncaught page error.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
import urllib.request
from pathlib import Path

from playwright.sync_api import Page, expect, sync_playwright

BASE = "http://127.0.0.1:8000"
API = f"{BASE}/api/v1/controller"


def api(path: str, body: dict | None = None):
    data = None if body is None else json.dumps(body).encode()
    request = urllib.request.Request(f"{API}{path}", data=data, headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(request, timeout=30) as response:
        return json.loads(response.read())


def status() -> dict:
    return api("/status")


def wait_until(predicate, timeout: float = 20.0, what: str = "condition"):
    deadline = time.monotonic() + timeout
    last = None
    while time.monotonic() < deadline:
        last = predicate()
        if last:
            return last
        time.sleep(0.05)
    raise TimeoutError(f"timed out waiting for {what}; last: {last!r}")


class Run:
    def __init__(self, page: Page, out: Path) -> None:
        self.page = page
        self.out = out
        self.step = 0

    def shot(self, name: str) -> None:
        self.step += 1
        self.page.screenshot(path=str(self.out / f"{self.step:02d}_{name}.png"), full_page=True)
        print(f"  ok: {name}", flush=True)

    def tab(self, name: str) -> None:
        self.page.get_by_test_id(f"tab-{name}").click()

    def click(self, test_id: str) -> None:
        self.page.get_by_test_id(test_id).click()


def choose(page: Page, trigger, option: str) -> None:
    """Pick from a Radix select: open it, then click the option by its text."""
    trigger.click()
    page.get_by_role("option", name=option, exact=True).click()


def run(page: Page, out: Path) -> None:
    r = Run(page, out)

    print("device", flush=True)
    page.goto(f"{BASE}/controller?tab=device")
    expect(page.get_by_test_id("run-bar")).to_be_visible()
    wait_until(lambda: page.get_by_test_id("port-row-0").count() > 0 or page.get_by_test_id("connect-button").count() > 0, what="device page")
    if page.get_by_test_id("connect-button").count():
        r.click("connect-button")
    expect(page.get_by_test_id("port-row-0")).to_be_visible(timeout=10000)
    r.click("discover-button")
    expect(page.get_by_test_id("port-row-0")).to_contain_text("reBot B601-RS", timeout=8000)
    expect(page.get_by_test_id("port-row-2")).to_contain_text("Star 102-HD")
    r.shot("device_discovered")

    # Port roles, chosen through the page.
    for port, role, model, serial in (
        (0, "左从臂", "reBot Arm B601-RS", "rs-left"),
        (1, "右从臂", "reBot Arm B601-RS", "rs-right"),
        (2, "左主臂", "Star Arm 102-HD", "hd-left"),
        (3, "右主臂", "Star Arm 102-HD", "hd-right"),
    ):
        row = page.get_by_test_id(f"port-row-{port}")
        selects = row.get_by_role("combobox")
        choose(page, selects.nth(0), role)
        choose(page, selects.nth(1), model)
        row.get_by_role("textbox").fill(serial)
    r.click("bind-button")
    wait_until(lambda: all(a.get("valid_mask") == 0x7F for a in status()["arms"]), what="all four arms reporting")
    r.shot("device_bound")

    # Cameras come from the one capture module, and the preview reads shared memory.
    r.click("start-cameras")
    page.wait_for_timeout(1500)
    wait_until(lambda: page.locator("img[alt='env']").count() > 0, what="preview element")
    page.wait_for_timeout(1500)
    natural = page.locator("img[alt='env']").evaluate("el => el.naturalWidth")
    assert natural > 0, "camera preview shows no image"
    r.shot("device_cameras")

    print("calibration", flush=True)
    r.tab("calibration")
    expect(page.get_by_test_id("capture-zero")).to_be_visible(timeout=5000)
    r.click("capture-zero")
    expect(page.get_by_text("✓").first).to_be_visible(timeout=5000)
    page.get_by_role("button", name="保存新版本").click()
    wait_until(lambda: next(a for a in status()["arms"] if a["arm"] == "left_follower").get("calibration_version") == 1, what="saved calibration")
    r.shot("calibration_saved")

    print("teleop", flush=True)
    r.tab("teleop")
    # Identity mapping, so the check below is about following and not about preset numbers.
    api("/teleop/config", {"pairs": [{"pair": i, "enabled": True, "scale": [1.0] * 7, "offset": [0.0] * 7} for i in (0, 1)]})
    r.click("teleop-enter")
    expect(page.get_by_test_id("control-source")).to_have_text("遥操", timeout=5000)
    api("/sim/inject", {"op": "SET_HAND_POSE", "arg": 2, "values": [0.5, 0.3, 0.3, 0.2, -0.2, 0.3, 0.5]})
    page.wait_for_timeout(1500)
    r.click("teleop-align")
    expect(page.get_by_test_id("pair-phase-0")).to_have_text("已对齐", timeout=15000)
    r.click("teleop-start")
    expect(page.get_by_test_id("pair-phase-0")).to_have_text("跟随中", timeout=5000)
    api("/sim/inject", {"op": "SET_HAND_POSE", "arg": 2, "values": [-0.2, 0.3, 0.3, 0.2, -0.2, 0.3, 0.5]})
    wait_until(lambda: abs(next(a for a in status()["arms"] if a["arm"] == "left_follower")["position"][0] + 0.2) < 0.06, what="follower follows leader")
    r.shot("teleop_following")
    r.click("teleop-end")
    expect(page.get_by_test_id("control-source")).to_have_text("无", timeout=5000)

    print("teaching", flush=True)
    r.tab("teach")
    canvas = page.get_by_test_id("robot-view").locator("canvas")
    expect(canvas).to_be_visible(timeout=10000)
    page.wait_for_timeout(5000)  # meshes
    r.shot("teach_3d")

    r.click("add-point")
    slider = page.get_by_role("slider", name="joint1")
    slider.evaluate("el => { const set = Object.getOwnPropertyDescriptor(HTMLInputElement.prototype, 'value').set; set.call(el, '0.4'); el.dispatchEvent(new Event('input', {bubbles:true})); el.dispatchEvent(new Event('change', {bubbles:true})); }")
    page.wait_for_timeout(200)
    r.click("add-point")
    page.wait_for_timeout(300)
    r.shot("teach_points")
    r.click("execute-program")
    expect(page.get_by_test_id("control-source")).to_have_text("示教", timeout=5000)
    wait_until(lambda: status()["source"] == "none", timeout=25, what="program finished")
    follower = next(a for a in status()["arms"] if a["arm"] == "left_follower")
    assert abs(follower["position"][0] - 0.4) < 0.08, f"program did not reach its point: {follower['position'][0]}"
    r.shot("teach_executed")

    print("recording", flush=True)
    r.tab("record")
    page.get_by_test_id("record-task").fill("pick the cube")
    page.get_by_label("相机（逗号分隔）").fill("env,wrist") if page.get_by_label("相机（逗号分隔）").count() else None
    expect(page.get_by_test_id("record-start")).to_be_enabled(timeout=5000)
    r.click("record-start")
    page.wait_for_timeout(500)

    print("deployment and takeover, while recording", flush=True)
    r.tab("policy")
    page.get_by_placeholder("例如：把方块放进盒子").fill("pick the cube")
    r.click("policy-start")
    expect(page.get_by_test_id("policy-state")).to_have_text("推理中", timeout=10000)
    wait_until(lambda: status()["policy"]["stats"].get("inferences", 0) >= 3, timeout=30, what="model process inferring")
    wait_until(lambda: status()["policy"]["stats"].get("actions_sent", 0) > 10, what="actions flowing")
    r.shot("policy_running")

    r.click("take-over")
    expect(page.get_by_test_id("policy-state")).to_have_text("已被接管", timeout=5000)
    expect(page.get_by_test_id("control-source")).to_have_text("人工纠正", timeout=5000)
    sent = status()["policy"]["stats"]["actions_sent"]
    page.wait_for_timeout(600)
    assert status()["policy"]["stats"]["actions_sent"] == sent, "actions kept flowing during a takeover"
    r.shot("policy_takeover")
    r.click("hand-back")
    expect(page.get_by_test_id("policy-state")).to_have_text("推理中", timeout=5000)
    wait_until(lambda: status()["policy"]["stats"]["actions_sent"] > sent + 5, timeout=15, what="model back in control")
    expect(page.get_by_test_id("intervention-count")).to_be_visible(timeout=3000)
    r.shot("policy_handed_back")
    r.click("policy-stop")
    expect(page.get_by_test_id("control-source")).to_have_text("无", timeout=5000)

    print("saving the recording", flush=True)
    r.tab("record")
    r.click("record-stop")
    expect(page.get_by_test_id("record-save")).to_be_visible(timeout=15000)
    r.shot("record_finished")
    r.click("record-save")
    expect(page.get_by_test_id("writer-job").first).to_have_attribute("data-state", "done", timeout=180000)
    r.shot("record_written")

    print("data browser", flush=True)
    r.tab("data")
    trigger = page.get_by_test_id("dataset-select")
    trigger.click()
    page.get_by_role("option").first.click()
    episode = page.get_by_test_id("episode-list").get_by_role("button").first
    expect(episode).to_be_visible(timeout=10000)
    expect(episode).to_contain_text("人工纠正")  # the automatic control-source label
    episode.click()
    video = page.get_by_test_id("video-env")
    expect(video).to_be_visible(timeout=10000)
    page.wait_for_function("document.querySelector('[data-testid=video-env]').readyState >= 2", timeout=20000)
    assert video.evaluate("el => el.videoWidth") > 0, "the recorded video does not decode in the browser"
    before = page.get_by_test_id("data-time").inner_text()
    r.click("data-play")
    page.wait_for_timeout(1000)
    r.click("data-play")
    after = page.get_by_test_id("data-time").inner_text()
    assert before != after, "playback did not advance"
    # Browsing is offline: the arm must not have been commanded by any of that.
    assert status()["source"] == "none"
    page.wait_for_timeout(500)
    r.shot("data_viewer")

    print("replay", flush=True)
    r.tab("replay")
    selects = page.get_by_role("combobox")
    selects.nth(0).click()
    page.get_by_role("option").first.click()
    page.wait_for_timeout(500)
    selects.nth(1).click()
    page.get_by_role("option").first.click()
    r.click("replay-start")
    expect(page.get_by_test_id("control-source")).to_have_text("回放", timeout=8000)
    page.wait_for_timeout(800)
    r.shot("replay_running")

    print("one-button stop and the emergency stop", flush=True)
    r.click("stop-button")
    expect(page.get_by_test_id("control-source")).to_have_text("无", timeout=5000)
    api("/sim/inject", {"op": "PRESS_ESTOP"})
    expect(page.get_by_test_id("estop-banner")).to_be_visible(timeout=3000)
    r.shot("estop_pressed")
    api("/sim/inject", {"op": "RELEASE_ESTOP"})
    reset = page.get_by_role("button", name="复位急停")
    expect(reset).to_be_visible(timeout=3000)
    reset.click()
    expect(page.get_by_test_id("estop-banner")).to_be_hidden(timeout=5000)
    r.shot("estop_reset")

    print("the lab's own launchpad still works", flush=True)
    page.goto(f"{BASE}/")
    expect(page.get_by_test_id("controller-entry")).to_be_visible(timeout=15000)
    r.shot("launchpad")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out", default="outputs/e2e", help="screenshot directory")
    parser.add_argument("--headed", action="store_true")
    args = parser.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    errors: list[str] = []
    with sync_playwright() as p:
        browser = None
        for channel in ("msedge", "chrome"):
            try:
                browser = p.chromium.launch(
                    channel=channel,
                    headless=not args.headed,
                    args=["--use-angle=swiftshader", "--enable-unsafe-swiftshader", "--ignore-gpu-blocklist"],
                )
                break
            except Exception as exc:
                print(f"{channel}: {exc}".splitlines()[0])
        if browser is None:
            print("no Edge or Chrome found")
            return 2
        context = browser.new_context(viewport={"width": 1500, "height": 1000}, locale="zh-CN")
        # The page's language is the one thing this test does not want to detect.
        context.add_init_script("try { localStorage.setItem('makerlab:language', 'zh-CN'); } catch {}")
        page = context.new_page()
        page.set_default_timeout(15000)
        page.on("pageerror", lambda e: errors.append(f"pageerror: {e}"))
        page.on("console", lambda m: errors.append(f"console: {m.text}") if m.type == "error" and "favicon" not in m.text else None)
        try:
            run(page, out)
        except Exception as exc:
            page.screenshot(path=str(out / "FAILED.png"), full_page=True)
            print(f"\nFAILED: {type(exc).__name__}: {str(exc)[:900]}")
            for line in errors[-12:]:
                print(" ", line[:300])
            return 1
        finally:
            browser.close()

    real = [e for e in errors if "409" not in e]
    if real:
        print(f"\npage reported {len(real)} errors:")
        for line in real[:20]:
            print(" ", line[:300])
        return 1
    print("\nall steps passed, no page errors")
    return 0


if __name__ == "__main__":
    sys.exit(main())
