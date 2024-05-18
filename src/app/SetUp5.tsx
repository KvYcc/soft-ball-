"use client";
import React, { useEffect, useRef } from "react";
import * as THREE from "three";
import * as CANNON from "cannon-es";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import Stats from "three/addons/libs/stats.module.js";

const SetUp: React.FC = () => {
  const ref = useRef<HTMLDivElement>(null);
  const statsDiv = useRef<HTMLDivElement>(null);

  interface SpringParams {
    restLength: number;
    stiffness: number;
    damping: number;
  }

  useEffect(() => {
    const currentRef = ref.current;
    const currentStatsDiv = statsDiv.current;
    if (!currentRef) return;

    //scene
    const scene = new THREE.Scene();

    //camera
    const camera = new THREE.PerspectiveCamera(
      75,
      window.innerWidth / window.innerHeight,
      0.1,
      1000
    );
    camera.position.set(5, 15, 5);
    camera.lookAt(0, 0, 0);

    //renderer
    const renderer = new THREE.WebGLRenderer();
    renderer.setSize(window.innerWidth, window.innerHeight);
    currentRef.appendChild(renderer.domElement);

    //light
    const light = new THREE.AmbientLight(0x404040);
    scene.add(light);
    const spotLight = new THREE.SpotLight("yellow", 100);
    // scene.add(spotLight);

    const lightHelper = new THREE.AmbientLight();
    scene.add(lightHelper);

    // grid helper
    const gridHelper = new THREE.GridHelper(50, 50);
    scene.add(gridHelper);

    //orbit control
    const orbit = new OrbitControls(camera, renderer.domElement);

    //stat
    const stats = new Stats();
    stats.showPanel(0); // 0: fps panel
    if (currentStatsDiv) {
      currentStatsDiv.appendChild(stats.dom);
    }
    //Cannon.js
    const world = new CANNON.World();
    world.gravity.set(0, -9.82, 0); // Set gravity in the world

    //broadphase
    world.broadphase = new CANNON.SAPBroadphase(world);
    world.allowSleep = true;

    //customSphere
    function createCustomSphere(scene: THREE.Scene, world: CANNON.World) {
      const radius = 1;
      const segments = 8; // 经度段
      const rings = 8; // 纬度段

      const sphereGeometry = new THREE.BufferGeometry();
      const vertices = [];
      const indices = [];

      // 添加顶点
      vertices.push(0, radius, 0); // 北极点

      for (let latNumber = 1; latNumber < rings; latNumber++) {
        const theta = (latNumber * Math.PI) / rings;
        const sinTheta = Math.sin(theta);
        const cosTheta = Math.cos(theta);

        for (let longNumber = 0; longNumber < segments; longNumber++) {
          const phi = (longNumber * 2 * Math.PI) / segments;
          const sinPhi = Math.sin(phi);
          const cosPhi = Math.cos(phi);

          const x = radius * sinTheta * cosPhi;
          const y = radius * cosTheta;
          const z = radius * sinTheta * sinPhi;
          vertices.push(x, y, z);
        }
      }

      vertices.push(0, -radius, 0); // 南极点

      // 连接顶点形成三角面
      for (let i = 1; i <= segments; i++) {
        indices.push(0, i, (i % segments) + 1);
      }

      let offset = 1;
      for (let latNumber = 0; latNumber < rings - 2; latNumber++) {
        for (let longNumber = 0; longNumber < segments; longNumber++) {
          const first = offset + longNumber + latNumber * segments;
          const second = first + segments;

          indices.push(
            first,
            second,
            longNumber === segments - 1
              ? offset + latNumber * segments
              : first + 1
          );

          if (longNumber < segments - 1) {
            indices.push(second, second + 1, first + 1);
          } else {
            indices.push(
              second,
              offset + (latNumber + 1) * segments,
              offset + latNumber * segments
            );
          }
        }
      }

      const bottomIndex = vertices.length / 3 - 1;
      const bottomStartIndex = bottomIndex - segments;
      for (let i = 0; i < segments; i++) {
        indices.push(
          bottomIndex,
          bottomStartIndex + i,
          bottomStartIndex + ((i + 1) % segments)
        );
      }

      sphereGeometry.setIndex(indices);
      sphereGeometry.setAttribute(
        "position",
        new THREE.Float32BufferAttribute(vertices, 3)
      );
      sphereGeometry.computeVertexNormals();

      const material = new THREE.MeshPhongMaterial({
        color: "purple",
        side: THREE.DoubleSide,
        specular: 0x555555, // 反射光的颜色
        shininess: 30,
        // wireframe: true,
        // 光泽度，数值越高反射越强
      });

      //   const material = new THREE.MeshBasicMaterial({
      //     color: 0xff0000,
      //     // wireframe: true,
      //     side: THREE.DoubleSide,
      //   });

      const sphereMesh2 = new THREE.Mesh(sphereGeometry, material);
      sphereMesh2.position.set(0, 5, 0);

      //   sphereMesh2.castShadow = true;
      //   sphereMesh2.receiveShadow = true;
      scene.add(sphereMesh2);

      const sphereShape = new CANNON.Sphere(radius);
      const sphereBody2 = new CANNON.Body({
        mass: 9,
        position: new CANNON.Vec3(0, 5, 0),
      });
      sphereBody2.addShape(sphereShape);
      //   world.addBody(sphereBody2);
      return { sphereMesh2, sphereBody2 };
    }

    const { sphereMesh2, sphereBody2 } = createCustomSphere(scene, world);

    //create particle, sp for short
    const radius = 0.0125;
    const massOfSParticle = 1;
    const sParticleGeo = new THREE.SphereGeometry(radius);
    const sParticleMat = new THREE.MeshBasicMaterial({ color: 0xffff00 });

    const sParticleMeshArray: THREE.Mesh[] = [];
    const sParticleBodyArray: CANNON.Body[] = [];
    const sParticles: { [key: string]: CANNON.Body } = {};

    const vertices = sphereMesh2.geometry.attributes.position;
    const numVertices = vertices.count;

    const quaternion = new CANNON.Quaternion();
    quaternion.setFromAxisAngle(new CANNON.Vec3(1, 0, 0), Math.PI / 2);

    const rotationMatrix = new THREE.Matrix4();
    rotationMatrix.makeRotationX(Math.PI / 2);

    for (let i = 0; i < numVertices; i++) {
      const x = vertices.getX(i) + sphereMesh2.position.x;
      const y = vertices.getY(i) + sphereMesh2.position.y;
      const z = vertices.getZ(i) + sphereMesh2.position.z;

      // Create a THREE.js vector for the position
      const positionVector = new THREE.Vector3(x, y, z);

      // Apply the rotation
      positionVector.applyMatrix4(rotationMatrix);

      // Create the CANNON.Body
      const sParticleBody = new CANNON.Body({
        mass: massOfSParticle,
        shape: new CANNON.Sphere(radius),
        position: new CANNON.Vec3(
          positionVector.x,
          positionVector.y,
          positionVector.z
        ),
        quaternion: quaternion.clone(), // Apply the rotation
      });
      sParticles[`${i}`] = sParticleBody;
      world.addBody(sParticleBody);
      sParticleBodyArray.push(sParticleBody);

      // Create the THREE.Mesh
      const sParticleMesh = new THREE.Mesh(sParticleGeo, sParticleMat);
      sParticleMesh.position.set(
        positionVector.x,
        positionVector.y,
        positionVector.z
      );
      sParticleMesh.quaternion.copy(quaternion); // Apply the rotation to the mesh
      scene.add(sParticleMesh);
      sParticleMeshArray.push(sParticleMesh);
      // console.log(sParticles);
    }

    //distanceCOnstraint
    function connectParticles(
      sParticles: { [key: string]: CANNON.Body } = {},
      mesh: THREE.Mesh
    ) {
      // Assuming mesh is a THREE.Mesh with THREE.SphereGeometry
      const geometry = mesh.geometry as THREE.SphereGeometry;

      // Check if the geometry has an index attribute
      if (geometry.index) {
        // const indices = geometry.index.array;
        console.log("gm", geometry);
        // Iterate through the indices two at a time to create constraints between vertices
        // for (let i = 0; i < indices.length; i += 3) {
        //   const indexA = indices[i];
        //   const indexB = indices[i + 1];
        //   const indexC = indices[i + 2];

        //loop version2

        // connectVertices(sParticles[indexA], sParticles[indexB]);
        // connectVertices(sParticles[indexB], sParticles[indexC]);
        // connectVertices(sParticles[indexC], sParticles[indexA]);
        //
        //first layer 米
        connectVertices(sParticles[0], sParticles[2]);
        connectVertices(sParticles[0], sParticles[3]);
        connectVertices(sParticles[0], sParticles[4]);
        connectVertices(sParticles[0], sParticles[5]);
        connectVertices(sParticles[0], sParticles[6]);
        connectVertices(sParticles[0], sParticles[7]);
        connectVertices(sParticles[0], sParticles[8]);
        connectVertices(sParticles[0], sParticles[1]);
        //first layer bot connect
        connectVertices(sParticles[1], sParticles[2]);
        connectVertices(sParticles[3], sParticles[2]);
        connectVertices(sParticles[4], sParticles[3]);
        connectVertices(sParticles[4], sParticles[5]);
        connectVertices(sParticles[6], sParticles[5]);
        connectVertices(sParticles[6], sParticles[7]);
        connectVertices(sParticles[7], sParticles[8]);
        connectVertices(sParticles[1], sParticles[8]);

        //second layer 米
        connectVertices(sParticles[10], sParticles[2]);
        connectVertices(sParticles[11], sParticles[3]);
        connectVertices(sParticles[12], sParticles[4]);
        connectVertices(sParticles[13], sParticles[5]);
        connectVertices(sParticles[14], sParticles[6]);
        connectVertices(sParticles[15], sParticles[7]);
        connectVertices(sParticles[16], sParticles[8]);
        connectVertices(sParticles[9], sParticles[1]);
        //second layer bot connect
        connectVertices(sParticles[9], sParticles[10]);
        connectVertices(sParticles[11], sParticles[10]);
        connectVertices(sParticles[12], sParticles[11]);
        connectVertices(sParticles[12], sParticles[13]);
        connectVertices(sParticles[14], sParticles[13]);
        connectVertices(sParticles[14], sParticles[15]);
        connectVertices(sParticles[15], sParticles[16]);
        connectVertices(sParticles[9], sParticles[16]);
        //second layer tri
        connectVertices(sParticles[3], sParticles[10]);
        connectVertices(sParticles[11], sParticles[4]);
        connectVertices(sParticles[12], sParticles[5]);
        connectVertices(sParticles[6], sParticles[13]);
        connectVertices(sParticles[14], sParticles[7]);
        connectVertices(sParticles[8], sParticles[15]);
        connectVertices(sParticles[1], sParticles[16]);
        connectVertices(sParticles[9], sParticles[2]);

        //third layer 米
        connectVertices(sParticles[18], sParticles[10]);
        connectVertices(sParticles[19], sParticles[11]);
        connectVertices(sParticles[20], sParticles[12]);
        connectVertices(sParticles[21], sParticles[13]);
        connectVertices(sParticles[22], sParticles[14]);
        connectVertices(sParticles[23], sParticles[15]);
        connectVertices(sParticles[24], sParticles[16]);
        connectVertices(sParticles[17], sParticles[9]);
        //third layer bot connect
        connectVertices(sParticles[17], sParticles[18]);
        connectVertices(sParticles[19], sParticles[18]);
        connectVertices(sParticles[20], sParticles[19]);
        connectVertices(sParticles[20], sParticles[21]);
        connectVertices(sParticles[22], sParticles[21]);
        connectVertices(sParticles[22], sParticles[23]);
        connectVertices(sParticles[23], sParticles[24]);
        connectVertices(sParticles[17], sParticles[24]);
        //third layer tri
        connectVertices(sParticles[11], sParticles[18]);
        connectVertices(sParticles[19], sParticles[12]);
        connectVertices(sParticles[20], sParticles[13]);
        connectVertices(sParticles[14], sParticles[21]);
        connectVertices(sParticles[22], sParticles[15]);
        connectVertices(sParticles[16], sParticles[23]);
        connectVertices(sParticles[9], sParticles[24]);
        connectVertices(sParticles[17], sParticles[10]);

        //fourth layer 米
        connectVertices(sParticles[26], sParticles[18]);
        connectVertices(sParticles[27], sParticles[19]);
        connectVertices(sParticles[28], sParticles[20]);
        connectVertices(sParticles[29], sParticles[21]);
        connectVertices(sParticles[30], sParticles[22]);
        connectVertices(sParticles[31], sParticles[23]);
        connectVertices(sParticles[32], sParticles[24]);
        connectVertices(sParticles[25], sParticles[17]);
        //fourth layer bot connect
        connectVertices(sParticles[25], sParticles[26]);
        connectVertices(sParticles[27], sParticles[26]);
        connectVertices(sParticles[28], sParticles[27]);
        connectVertices(sParticles[28], sParticles[29]);
        connectVertices(sParticles[30], sParticles[29]);
        connectVertices(sParticles[30], sParticles[31]);
        connectVertices(sParticles[31], sParticles[32]);
        connectVertices(sParticles[25], sParticles[32]);
        //fourth layer tri
        connectVertices(sParticles[19], sParticles[26]);
        connectVertices(sParticles[27], sParticles[20]);
        connectVertices(sParticles[28], sParticles[21]);
        connectVertices(sParticles[22], sParticles[29]);
        connectVertices(sParticles[30], sParticles[23]);
        connectVertices(sParticles[24], sParticles[31]);
        connectVertices(sParticles[17], sParticles[32]);
        connectVertices(sParticles[25], sParticles[18]);

        //   fifth layer 米
        connectVertices(sParticles[34], sParticles[26]);
        connectVertices(sParticles[35], sParticles[27]);
        connectVertices(sParticles[36], sParticles[28]);
        connectVertices(sParticles[37], sParticles[29]);
        connectVertices(sParticles[38], sParticles[30]);
        connectVertices(sParticles[39], sParticles[31]);
        connectVertices(sParticles[40], sParticles[32]);
        connectVertices(sParticles[33], sParticles[25]);
        //fifth layer bot connect
        connectVertices(sParticles[33], sParticles[34]);
        connectVertices(sParticles[35], sParticles[34]);
        connectVertices(sParticles[36], sParticles[35]);
        connectVertices(sParticles[36], sParticles[37]);
        connectVertices(sParticles[38], sParticles[37]);
        connectVertices(sParticles[38], sParticles[39]);
        connectVertices(sParticles[39], sParticles[40]);
        connectVertices(sParticles[33], sParticles[40]);
        //fifth layer tri
        connectVertices(sParticles[27], sParticles[34]);
        connectVertices(sParticles[35], sParticles[28]);
        connectVertices(sParticles[36], sParticles[29]);
        connectVertices(sParticles[30], sParticles[37]);
        connectVertices(sParticles[38], sParticles[31]);
        connectVertices(sParticles[32], sParticles[39]);
        connectVertices(sParticles[25], sParticles[40]);
        connectVertices(sParticles[33], sParticles[26]);

        //   sixth layer 米
        connectVertices(sParticles[42], sParticles[34]);
        connectVertices(sParticles[43], sParticles[35]);
        connectVertices(sParticles[44], sParticles[36]);
        connectVertices(sParticles[45], sParticles[37]);
        connectVertices(sParticles[46], sParticles[38]);
        connectVertices(sParticles[47], sParticles[39]);
        connectVertices(sParticles[48], sParticles[40]);
        connectVertices(sParticles[41], sParticles[33]);
        //sixth layer bot connect
        connectVertices(sParticles[41], sParticles[42]);
        connectVertices(sParticles[43], sParticles[42]);
        connectVertices(sParticles[44], sParticles[43]);
        connectVertices(sParticles[44], sParticles[45]);
        connectVertices(sParticles[46], sParticles[45]);
        connectVertices(sParticles[46], sParticles[47]);
        connectVertices(sParticles[47], sParticles[48]);
        connectVertices(sParticles[41], sParticles[48]);
        //   sixth layer tri
        connectVertices(sParticles[35], sParticles[42]);
        connectVertices(sParticles[43], sParticles[36]);
        connectVertices(sParticles[44], sParticles[37]);
        connectVertices(sParticles[38], sParticles[45]);
        connectVertices(sParticles[46], sParticles[39]);
        connectVertices(sParticles[40], sParticles[47]);
        connectVertices(sParticles[33], sParticles[48]);
        connectVertices(sParticles[41], sParticles[34]);
        //seventh layer 米
        connectVertices(sParticles[50], sParticles[42]);
        connectVertices(sParticles[51], sParticles[43]);
        connectVertices(sParticles[52], sParticles[44]);
        connectVertices(sParticles[53], sParticles[45]);
        connectVertices(sParticles[54], sParticles[46]);
        connectVertices(sParticles[55], sParticles[47]);
        connectVertices(sParticles[56], sParticles[48]);
        connectVertices(sParticles[49], sParticles[41]);
        // seventh layer bot connect
        connectVertices(sParticles[49], sParticles[50]);
        connectVertices(sParticles[51], sParticles[50]);
        connectVertices(sParticles[52], sParticles[51]);
        connectVertices(sParticles[52], sParticles[53]);
        connectVertices(sParticles[54], sParticles[53]);
        connectVertices(sParticles[54], sParticles[55]);
        connectVertices(sParticles[55], sParticles[56]);
        connectVertices(sParticles[49], sParticles[56]);
        //seventh layer tri
        connectVertices(sParticles[43], sParticles[50]);
        connectVertices(sParticles[51], sParticles[44]);
        connectVertices(sParticles[52], sParticles[45]);
        connectVertices(sParticles[46], sParticles[53]);
        connectVertices(sParticles[54], sParticles[47]);
        connectVertices(sParticles[48], sParticles[55]);
        connectVertices(sParticles[41], sParticles[56]);
        connectVertices(sParticles[49], sParticles[42]);

        //pole layer米
        connectVertices(sParticles[57], sParticles[49]);
        connectVertices(sParticles[57], sParticles[50]);
        connectVertices(sParticles[57], sParticles[51]);
        connectVertices(sParticles[57], sParticles[52]);
        connectVertices(sParticles[57], sParticles[53]);
        connectVertices(sParticles[57], sParticles[54]);
        connectVertices(sParticles[57], sParticles[55]);
        connectVertices(sParticles[57], sParticles[56]);
        //pole layer bot
        connectVertices(sParticles[49], sParticles[50]);
        connectVertices(sParticles[50], sParticles[51]);
        connectVertices(sParticles[51], sParticles[52]);
        connectVertices(sParticles[52], sParticles[53]);
        connectVertices(sParticles[53], sParticles[54]);
        connectVertices(sParticles[54], sParticles[55]);
        connectVertices(sParticles[55], sParticles[56]);
        connectVertices(sParticles[56], sParticles[49]);

        // }
      }
    }
    connectParticles(sParticles, sphereMesh2);

    //connectVertice

    function connectVertices(particleA: CANNON.Body, particleB: CANNON.Body) {
      const distance = particleA.position.distanceTo(particleB.position);

      if (distance > 0) {
        const constraint = new CANNON.DistanceConstraint(
          particleA,
          particleB,
          distance
        );

        world.addConstraint(constraint);
      }
    }

    //spring
    function particleSpring(
      particles: { [key: string]: CANNON.Body } = {},
      center: CANNON.Body,
      springParams: SpringParams
    ) {
      const springs: CANNON.Spring[] = [];
      Object.values(particles).forEach((particle) => {
        const spring = new CANNON.Spring(center, particle, {
          restLength: springParams.restLength,
          stiffness: springParams.stiffness,
          damping: springParams.damping,
        });
        springs.push(spring);
      });
      return springs;
    }

    // center body
    const centerBody = new CANNON.Body({
      mass: 5,
      position: new CANNON.Vec3(0, 5, 0),
    });

    world.addBody(centerBody);

    const springs = particleSpring(sParticles, centerBody, {
      restLength: 0.4,
      stiffness: 0,
      damping: 0,
    });

    // sphere
    function updateSphereVertices(
      sphereMesh: THREE.Mesh,
      particleBodies: { [key: string]: CANNON.Body }
    ) {
      const vertices = sphereMesh.geometry.attributes.position;
      const bodies = Object.values(particleBodies);

      for (let i = 0; i < bodies.length && i < vertices.count; i++) {
        const body = bodies[i];
        vertices.setXYZ(
          i,
          body.position.x - sphereMesh.position.x,
          body.position.y - sphereMesh.position.y,
          body.position.z - sphereMesh.position.z
        );
      }

      vertices.needsUpdate = true;
      sphereMesh.geometry.computeVertexNormals();
    }

    //ground mesh
    const massGround: number = 0;
    const groundGeo = new THREE.PlaneGeometry(30, 30);
    const groundMat = new THREE.MeshBasicMaterial({
      color: 0xffffff,
      side: THREE.DoubleSide,
      wireframe: true,
    });
    const groundMesh = new THREE.Mesh(groundGeo, groundMat);
    groundMesh.rotateX(-Math.PI / 2);
    scene.add(groundMesh);

    //ground body
    const groundBody = new CANNON.Body({
      shape: new CANNON.Plane(),
      mass: massGround,
      // wireframe: true
    });
    groundBody.quaternion.setFromAxisAngle(
      new CANNON.Vec3(1, 0, 0),
      -Math.PI / 2
    );
    world.addBody(groundBody);

    // world.addEventListener("postStep", () => {
    //   springs.forEach((spring) => {
    //     spring.applyForce();
    //   });
    // });

    //Resize
    const onWindowResize = () => {
      camera.aspect = window.innerWidth / window.innerHeight;
      camera.updateProjectionMatrix();
      renderer.setSize(window.innerWidth, window.innerHeight);
    };
    window.addEventListener("resize", onWindowResize);

    // Function to visualize constraints
    function visualizeConstraints(
      particles: { [key: string]: CANNON.Body } = {},
      constraints: CANNON.Constraint[],
      scene: THREE.Scene
    ) {
      // Material for the constraint lines
      const material = new THREE.LineBasicMaterial({ color: 0x00ff00 });

      // Remove previous lines if any
      scene.children = scene.children.filter(
        (child) => !(child instanceof THREE.Line)
      );

      // Iterate over each constraint

      Object.values(constraints).forEach((constraint) => {
        const geometry = new THREE.BufferGeometry().setFromPoints([
          new THREE.Vector3(
            constraint.bodyA.position.x,
            constraint.bodyA.position.y,
            constraint.bodyA.position.z
          ),
          new THREE.Vector3(
            constraint.bodyB.position.x,
            constraint.bodyB.position.y,
            constraint.bodyB.position.z
          ),
        ]);

        // Create a line to represent the constraint
        const line = new THREE.Line(geometry, material);
        scene.add(line);
      });
    }
    // visualizeConstraints(
    //   sParticles,
    //   world.constraints as CANNON.Constraint[],
    //   scene
    // );

    // Animation loop
    const animate = () => {
      requestAnimationFrame(animate);
      //   springs.forEach((spring) => spring.applyForce());
      updateSphereVertices(sphereMesh2, sParticles);

      // Update physics world
      world.step(1 / 60);
      //   sphereMesh2.position.copy(sphereBody2.position);
      //   sphereMesh2.quaternion.copy(sphereBody2.quaternion);
      stats.begin();

      stats.end();
      // Render the scene
      renderer.render(scene, camera);

      for (let i = 0; i < sParticleBodyArray.length; i++) {
        sParticleMeshArray[i].position.copy(
          sParticleBodyArray[i].position as any
        );
        sParticleMeshArray[i].quaternion.copy(
          sParticleBodyArray[i].quaternion as any
        );
      }
    };
    animate();

    return () => {
      window.removeEventListener("resize", onWindowResize);
      if (currentRef) {
        currentRef.removeChild(renderer.domElement);
      }
      renderer.dispose(); // Clean up resources
    };
  }, []);

  return (
    <div ref={ref} className="w-full h-screen">
      <div ref={statsDiv} className="'stats"></div>
    </div>
  );
};

export default SetUp;
