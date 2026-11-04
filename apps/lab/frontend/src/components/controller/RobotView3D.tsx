/**
 * 3D arm view for teaching: the solid model is measured feedback, the translucent
 * one is the target, the same split the ReBot RS Digital Twin documents. Joints can
 * be dragged on the target model, and the handle drags the end effector. Inverse
 * kinematics runs on the computer: this component only reports the requested pose.
 */
import React, { useEffect, useRef, useState } from "react";
import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import { TransformControls } from "three/examples/jsm/controls/TransformControls.js";
import { STLLoader } from "three/examples/jsm/loaders/STLLoader.js";
import URDFLoader, { URDFRobot } from "urdf-loader";
import { PointerURDFDragControls } from "urdf-loader/src/URDFDragControls.js";
import { useApi } from "@/contexts/ApiContext";
import { useController } from "@/contexts/ControllerContext";

const ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"];

/** Normalized gripper travel to the model's prismatic fingers, where the URDF has them. */
function applyGripper(robot: URDFRobot, value: number) {
  for (const [name, joint] of Object.entries(robot.joints)) {
    if (name.startsWith("gripper_joint") && joint.limit) {
      joint.setJointValue(Number(joint.limit.upper) * value);
    }
  }
}

function applyPose(robot: URDFRobot | null, pose: number[] | undefined) {
  if (!robot || !pose) return;
  ARM_JOINTS.forEach((name, index) => robot.joints[name]?.setJointValue(pose[index]));
  applyGripper(robot, pose[6] ?? 0);
}

function makeGhost(root: THREE.Object3D) {
  const material = new THREE.MeshStandardMaterial({ color: 0x4aa8ff, transparent: true, opacity: 0.32, depthWrite: false });
  root.traverse((child) => {
    if ((child as THREE.Mesh).isMesh) (child as THREE.Mesh).material = material;
  });
}

interface Props {
  arm: string;
  urdf: string | null;
  endEffector: string | null;
  /** Live target, or a preview pose while a program preview runs. */
  target: number[] | null;
  onJointDrag?: (joint: string, value: number) => void;
  onHandleDrag?: (xyz: number[], rpy: number[]) => void;
  onDragEnd?: () => void;
  interactive: boolean;
}

const RobotView3D: React.FC<Props> = ({ arm, urdf, endEffector, target, onJointDrag, onHandleDrag, onDragEnd, interactive }) => {
  const { baseUrl } = useApi();
  const { samplesRef } = useController();
  const container = useRef<HTMLDivElement>(null);
  const targetRef = useRef<number[] | null>(target);
  const interactiveRef = useRef(interactive);
  const callbacks = useRef({ onJointDrag, onHandleDrag, onDragEnd });
  const [error, setError] = useState<string | null>(null);

  targetRef.current = target;
  interactiveRef.current = interactive;
  callbacks.current = { onJointDrag, onHandleDrag, onDragEnd };

  useEffect(() => {
    const host = container.current;
    if (!host || !urdf || !endEffector) return;
    setError(null);

    let renderer: THREE.WebGLRenderer;
    try {
      renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    } catch {
      setError("WebGL is not available in this browser.");
      return;
    }
    renderer.setPixelRatio(window.devicePixelRatio);
    host.appendChild(renderer.domElement);

    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x14171c);
    scene.add(new THREE.HemisphereLight(0xffffff, 0x333340, 1.6));
    const sun = new THREE.DirectionalLight(0xffffff, 1.4);
    sun.position.set(1, 2, 2);
    scene.add(sun);
    const grid = new THREE.GridHelper(1.4, 14, 0x3a4048, 0x262a30);
    scene.add(grid);

    const camera = new THREE.PerspectiveCamera(45, 1, 0.01, 20);
    camera.position.set(0.75, 0.6, 0.75);
    const orbit = new OrbitControls(camera, renderer.domElement);
    orbit.target.set(0, 0.22, 0);
    orbit.update();

    // URDF is Z-up, the scene is Y-up.
    const world = new THREE.Group();
    world.rotation.x = -Math.PI / 2;
    scene.add(world);

    let measured: URDFRobot | null = null;
    let ghost: URDFRobot | null = null;
    let disposed = false;

    const manager = new THREE.LoadingManager();
    const loader = new URDFLoader(manager);
    loader.packages = "";
    loader.loadMeshCb = (path, meshManager, done) => {
      new STLLoader(meshManager).load(
        path,
        (geometry) => done(new THREE.Mesh(geometry, new THREE.MeshStandardMaterial({ color: 0xc9ced6, metalness: 0.2, roughness: 0.6 }))),
        undefined,
        () => done(new THREE.Object3D()),
      );
    };

    const url = `${baseUrl}/robots/${urdf}`;
    const handle = new THREE.Object3D();
    world.add(handle);
    const transform = new TransformControls(camera, renderer.domElement);
    transform.setSize(0.7);
    transform.setSpace("world");
    transform.attach(handle);
    const gizmo = transform.getHelper();
    scene.add(gizmo);

    let dragging = false;
    transform.addEventListener("dragging-changed", (event) => {
      dragging = !!event.value;
      orbit.enabled = !dragging;
      if (!dragging) callbacks.current.onDragEnd?.();
    });
    transform.addEventListener("objectChange", () => {
      if (!dragging) return;
      // The handle sits under `world`, whose frame is the URDF base frame.
      const q = handle.quaternion;
      const euler = new THREE.Euler().setFromQuaternion(q, "ZYX");
      callbacks.current.onHandleDrag?.(handle.position.toArray() as number[], [euler.x, euler.y, euler.z]);
    });

    let dragControls: PointerURDFDragControls | null = null;

    loader.load(
      url,
      (robot) => {
        if (disposed) return;
        measured = robot;
        world.add(robot);
      },
      undefined,
      () => setError(`Could not load ${url}`),
    );
    const ghostLoader = new URDFLoader(manager);
    ghostLoader.loadMeshCb = loader.loadMeshCb;
    ghostLoader.load(url, (robot) => {
      if (disposed) return;
      ghost = robot;
      world.add(robot);
      dragControls = new PointerURDFDragControls(robot, camera, renderer.domElement);
      dragControls.updateJoint = (joint: { name: string }, angle: number) => {
        if (interactiveRef.current && ARM_JOINTS.includes(joint.name)) {
          callbacks.current.onJointDrag?.(joint.name, angle);
        }
      };
      dragControls.onDragStart = () => {
        orbit.enabled = false;
      };
      dragControls.onDragEnd = () => {
        orbit.enabled = true;
        callbacks.current.onDragEnd?.();
      };
    });
    manager.onLoad = () => {
      if (ghost) makeGhost(ghost);
    };

    const tip = new THREE.Vector3();
    const tipQ = new THREE.Quaternion();
    const worldInverse = new THREE.Matrix4();
    const tipMatrix = new THREE.Matrix4();

    let frame = 0;
    const resize = () => {
      const { clientWidth, clientHeight } = host;
      renderer.setSize(clientWidth, clientHeight);
      camera.aspect = clientWidth / Math.max(1, clientHeight);
      camera.updateProjectionMatrix();
    };
    const observer = new ResizeObserver(resize);
    observer.observe(host);
    resize();

    const animate = () => {
      frame = requestAnimationFrame(animate);
      const sample = samplesRef.current[arm];
      applyPose(measured, sample?.position);
      const wanted = targetRef.current ?? sample?.target ?? sample?.position;
      applyPose(ghost, wanted);

      transform.enabled = interactiveRef.current;
      gizmo.visible = interactiveRef.current;
      if (ghost && !dragging) {
        const link = ghost.links[endEffector] ?? ghost.frames?.[endEffector];
        if (link) {
          ghost.updateMatrixWorld(true);
          worldInverse.copy(world.matrixWorld).invert();
          tipMatrix.multiplyMatrices(worldInverse, link.matrixWorld);
          tipMatrix.decompose(tip, tipQ, new THREE.Vector3());
          handle.position.copy(tip);
          handle.quaternion.copy(tipQ);
        }
      }
      renderer.render(scene, camera);
    };
    animate();

    return () => {
      disposed = true;
      cancelAnimationFrame(frame);
      observer.disconnect();
      dragControls?.dispose?.();
      transform.detach();
      transform.dispose();
      orbit.dispose();
      renderer.dispose();
      renderer.domElement.remove();
    };
  }, [arm, urdf, endEffector, baseUrl, samplesRef]);

  return (
    <div ref={container} className="relative h-[460px] w-full overflow-hidden rounded border" data-testid="robot-view">
      {error && <div className="absolute inset-0 flex items-center justify-center p-4 text-sm text-destructive">{error}</div>}
    </div>
  );
};

export default RobotView3D;
