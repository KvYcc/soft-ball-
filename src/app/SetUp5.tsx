"use client";
import React, { useEffect, useRef } from "react";
import * as THREE from "three";
import * as CANNON from "cannon-es";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import Stats from "three/addons/libs/stats.module.js";
import { createBouncingBall } from './Ball'
import Soccer from "../../public/Soccer.jpg";


const SetUp: React.FC = () => {
  const ref = useRef<HTMLDivElement>(null);
  const statsDiv = useRef<HTMLDivElement>(null);

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
    const updateCamera = () => {
      let avgX = 0;
      let avgY = 0;
      let avgZ = 0;
      const numParticles = sParticleBodyArray.length;

      for (let i = 0; i < numParticles; i++) {
        const position = sParticleBodyArray[i].position;
        avgX += position.x;
        avgY += position.y;
        avgZ += position.z;
      }

      avgX /= numParticles;
      avgY /= numParticles;
      avgZ /= numParticles;

      camera.position.set(avgX , avgY + 10, avgZ + 10);
      camera.lookAt(avgX, avgY, avgZ);
    };
    // camera.position.set(5, 15, 5);
    // camera.lookAt(0, 0, 0);

    //renderer
    const renderer = new THREE.WebGLRenderer();
    renderer.setSize(window.innerWidth, window.innerHeight);
    currentRef.appendChild(renderer.domElement);

    //light
    const light = new THREE.AmbientLight(0x404040);
    scene.add(light);
    const spotLight = new THREE.SpotLight("white", 100);
    scene.add(spotLight);

    const lightHelper = new THREE.AmbientLight();
    scene.add(lightHelper);

    // grid helper
    const gridHelper = new THREE.GridHelper(50, 50);
    scene.add(gridHelper);

    //orbit control
    const orbit = new OrbitControls(camera, renderer.domElement);

    //stat
    const stats = new Stats();
    stats.showPanel(0);
    if (currentStatsDiv) {
      currentStatsDiv.appendChild(stats.dom);
    }
    //Cannon.js
    const world = new CANNON.World();
    world.gravity.set(0, -9.82, 0);

    //broadphase
    world.broadphase = new CANNON.SAPBroadphase(world);
    world.allowSleep = true;

    //customSphere
    function createCustomSphere(scene: THREE.Scene, world: CANNON.World) {
      const radius = 1;
      const segments = 8;
      const rings = 8;

      const sphereGeometry = new THREE.BufferGeometry();
      const vertices = [];
      const indices = [];

      vertices.push(0, radius, 0);

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
      vertices.push(0, -radius, 0);

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

      const textureLoader = new THREE.TextureLoader()
      const texture = textureLoader.load(Soccer.src, ()=>{
        renderer.render(scene, camera);
      })
      console.log(texture)
      const material = new THREE.MeshBasicMaterial({
        map:texture,
        side: THREE.DoubleSide,
        // wireframe: true,
      });

      //   const material = new THREE.MeshBasicMaterial({
      //     color: 0xff0000,
      //     // wireframe: true,
      //     side: THREE.DoubleSide,
      //   });

      const sphereMesh2 = new THREE.Mesh(sphereGeometry, material);
      sphereMesh2.position.set(0, 5, 0);

      sphereMesh2.castShadow = true;
      sphereMesh2.receiveShadow = true;
      scene.add(sphereMesh2);

      const sphereShape = new CANNON.Sphere(radius);
      const sphereBody2 = new CANNON.Body({
        mass: 0,
        position: new CANNON.Vec3(0, 5, 0),
      });
      sphereBody2.addShape(sphereShape);
      // world.addBody(sphereBody2);
      return { sphereMesh2, sphereBody2 };
    }

    const { sphereMesh2, sphereBody2 } = createCustomSphere(scene, world);

    const { update: updateBouncingBall } = createBouncingBall(
      scene,
      world,
      1, 
      new THREE.Vector3(5, 1, 0), 
      new THREE.Vector3(-10, 0, 0), 
      1 
    );

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

    // const rotationMatrix = new THREE.Matrix4();
    // rotationMatrix.makeRotationX(Math.PI / 2);

    for (let i = 0; i < numVertices; i++) {
      const x = vertices.getX(i) + sphereMesh2.position.x;
      const y = vertices.getY(i) + sphereMesh2.position.y;
      const z = vertices.getZ(i) + sphereMesh2.position.z;

      // Create a THREE.js vector for the position
      const positionVector = new THREE.Vector3(x, y, z);

      // Apply the rotation
      // positionVector.applyMatrix4(rotationMatrix);

      // Create the CANNON.Body
      const sParticleBody = new CANNON.Body({
        mass: massOfSParticle,
        shape: new CANNON.Sphere(radius),
        position: new CANNON.Vec3(
          positionVector.x,
          positionVector.y,
          positionVector.z
        ),
        quaternion: quaternion.clone(), 
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
      sParticleMesh.quaternion.copy(quaternion); 
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

    //wall
    const massWall = 0;
    const wallWidth = 10;
    const wallHeight = 10;
    const wallDepth = 1;
    const wallGeo = new THREE.BoxGeometry(wallWidth, wallHeight, wallDepth);
    const wallMat = new THREE.MeshBasicMaterial({
      color: 0xffffff,
      side: THREE.DoubleSide,
      opacity:0.1,
      transparent:true
      // wireframe: true,
    });
    const wallMesh = new THREE.Mesh(wallGeo, wallMat);
    scene.add(wallMesh);
    wallMesh.position.set(0, 0, 10);

    //body
    const wallShape = new CANNON.Box(
      new CANNON.Vec3(wallWidth / 2, wallHeight / 2, wallDepth / 2)
    );
    const wallBody = new CANNON.Body({
      mass: massWall,
    });
    wallBody.addShape(wallShape);
    wallBody.position.set(0, 0, 10);
    world.addBody(wallBody);

    const keysFunction = {
      w: false,
      a: false,
      s: false,
      d: false,
      space: false,
    };
    const forceMagnitude = 5;
    const torqueMagnitude = 1;
    const jumpForce = 5;

    const handleKeyDown = (event: KeyboardEvent) => {
      switch (event.key) {
        case "w":
          keysFunction.w = true;
          break;
        case "a":
          keysFunction.a = true;
          break;
        case "s":
          keysFunction.s = true;
          break;
        case "d":
          keysFunction.d = true;
          break;
        case " ":
          keysFunction.space = true;
          break;
      }
    };

    const handleKeyUp = (event: KeyboardEvent) => {
      switch (event.key) {
        case "w":
          keysFunction.w = false;
          break;
        case "a":
          keysFunction.a = false;
          break;
        case "s":
          keysFunction.s = false;
          break;
        case "d":
          keysFunction.d = false;
          break;
        case " ":
          keysFunction.space = false;
          break;
      }
    };

    window.addEventListener("keydown", handleKeyDown);
    window.addEventListener("keyup", handleKeyUp);

    const applyForces = () => {
      for (let i = 0; i < sParticleBodyArray.length; i++) {
        const body = sParticleBodyArray[i];

        if (keysFunction.w) {
          body.applyForce(
            new CANNON.Vec3(0, 0, -forceMagnitude),
            body.position
          );
          body.applyTorque(new CANNON.Vec3(-torqueMagnitude, 0, 0));
        }
        if (keysFunction.s) {
          body.applyForce(new CANNON.Vec3(0, 0, forceMagnitude), body.position);
          body.applyTorque(new CANNON.Vec3(torqueMagnitude, 0, 0));
        }
        if (keysFunction.a) {
          body.applyForce(
            new CANNON.Vec3(-forceMagnitude, 0, 0),
            body.position
          );
          body.applyTorque(new CANNON.Vec3(0, 0, torqueMagnitude));
        }
        if (keysFunction.d) {
          body.applyForce(new CANNON.Vec3(forceMagnitude, 0, 0), body.position);
          body.applyTorque(new CANNON.Vec3(0, 0, -torqueMagnitude));
        }
        if (keysFunction.space) {
          body.velocity.set(0, body.velocity.y, 0);
          body.angularVelocity.set(0, 0, 0);
          body.applyImpulse(new CANNON.Vec3(0, jumpForce, 0), body.position);
        }

        // scale to reduce damping
        body.velocity.scale(0.95, body.velocity);
        body.angularVelocity.scale(0.95, body.angularVelocity);
      }
    };

    //Resize
    const onWindowResize = () => {
      camera.aspect = window.innerWidth / window.innerHeight;
      camera.updateProjectionMatrix();
      renderer.setSize(window.innerWidth, window.innerHeight);
    };
    window.addEventListener("resize", onWindowResize);

    // Animation loop
    const animate = () => {
      requestAnimationFrame(animate);
      // springs.forEach((spring) => spring.applyForce());

      // Update physics world
      //   sphereMesh2.position.copy(sphereBody2.position);
      //   sphereMesh2.quaternion.copy(sphereBody2.quaternion);

      //force
      // const force = 1;
      // const direction = new CANNON.Vec3(0,0,0);
      // if (keysFunction.w) {
      //   direction.z -= force;
      // }
      // if (keysFunction.a) {
      //   direction.x -= force;
      // }
      // if (keysFunction.s) {
      //   direction.z += force;
      // }
      // if (keysFunction.d) {
      //   direction.x += force;
      // }
      applyForces();
      world.step(1 / 60);

      updateSphereVertices(sphereMesh2, sParticles);

      updateBouncingBall();

      updateCamera();

      // sphereBody2.applyLocalForce(direction, new CANNON.Vec3(0, 0, 0))

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
    const wallThickness = 1;
    const wallHeight2 = 10;
    const wallLength = 50;
    const wallMaterial = new THREE.MeshBasicMaterial({
      color: 0xffffff,
      side: THREE.DoubleSide,
      opacity: 0.1,
      transparent: true
    });

    // 墙体几何体
    const createWall = (width: number, height: number, depth: number) => {
      return new THREE.BoxGeometry(width, height, depth);
    };

    // 墙体的位置
    const createWallMesh = (
      geometry: THREE.BoxGeometry,
      x: number,
      y: number,
      z: number,
      rotationY: number
    ) => {
      const wallMesh = new THREE.Mesh(geometry, wallMaterial);
      wallMesh.position.set(x, y, z);
      wallMesh.rotation.y = rotationY;
      return wallMesh;
    };

    // 创建并添加四面墙体
    const wallGeometry1 = createWall(wallLength, wallHeight2, wallThickness); // 南北向墙
    const wallGeometry2 = createWall(wallThickness, wallHeight2, wallLength); // 东西向墙

    const wall1 = createWallMesh(
      wallGeometry1,
      0,
      wallHeight2 / 2,
      wallLength / 2,
      0
    ); // 北墙
    const wall2 = createWallMesh(
      wallGeometry1,
      0,
      wallHeight2 / 2,
      -wallLength / 2,
      0
    ); // 南墙
    const wall3 = createWallMesh(
      wallGeometry2,
      wallLength / 2,
      wallHeight2 / 2,
      0,
      Math.PI
    ); // 东墙
    const wall4 = createWallMesh(
      wallGeometry2,
      -wallLength / 2,
      wallHeight2 / 2,
      0,
      Math.PI
    ); // 西墙

    scene.add(wall1);
    scene.add(wall2);
    scene.add(wall3);
    scene.add(wall4);

    // 添加物理墙体
    const addPhysicalWall = (
      width: number,
      height: number,
      depth: number,
      x: number,
      y: number,
      z: number
    ) => {
      const shape = new CANNON.Box(
        new CANNON.Vec3(width / 2, height / 2, depth / 2)
      );
      const body = new CANNON.Body({
        mass: 0, // 静止物体
        position: new CANNON.Vec3(x, y, z),
        shape: shape,
      });
      world.addBody(body);
    };

    addPhysicalWall(
      wallLength,
      wallHeight2,
      wallThickness,
      0,
      wallHeight2 / 2,
      wallLength / 2
    );
    addPhysicalWall(
      wallLength,
      wallHeight2,
      wallThickness,
      0,
      wallHeight2 / 2,
      -wallLength / 2
    );
    addPhysicalWall(
      wallThickness,
      wallHeight2,
      wallLength,
      wallLength / 2,
      wallHeight2 / 2,
      0
    );
    addPhysicalWall(
      wallThickness,
      wallHeight2,
      wallLength,
      -wallLength / 2,
      wallHeight2 / 2,
      0
    );

    return () => {
      window.removeEventListener("resize", onWindowResize);
      window.removeEventListener("keydown", handleKeyDown);
      window.removeEventListener("keyup", handleKeyUp);
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
