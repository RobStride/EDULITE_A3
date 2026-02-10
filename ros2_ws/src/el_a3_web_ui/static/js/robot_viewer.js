/**
 * EL-A3 Web UI - 3D Robot Viewer
 * Three.js based URDF robot model visualization with real STL meshes
 */

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';

class RobotViewer {
    constructor(containerId) {
        this.container = document.getElementById(containerId);
        this.scene = null;
        this.camera = null;
        this.renderer = null;
        this.controls = null;
        this.robot = null;
        this.gridHelper = null;
        this.jointGroups = {};
        this.showGrid = true;
        this.stlLoader = new STLLoader();
        this.loadingCount = 0;
        this.totalMeshes = 0;
        this.loadedMeshes = [];  // For debugging
        
        // Material colors matching URDF - with DoubleSide rendering
        this.materials = {
            orange: new THREE.MeshPhongMaterial({ color: 0xf88700, side: THREE.DoubleSide }),
            gray: new THREE.MeshPhongMaterial({ color: 0xa5a5a5, side: THREE.DoubleSide }),
            light_blue: new THREE.MeshPhongMaterial({ color: 0xc4e3f3, side: THREE.DoubleSide }),
            white: new THREE.MeshPhongMaterial({ color: 0xeaeaea, side: THREE.DoubleSide }),
            blue: new THREE.MeshPhongMaterial({ color: 0x3b6099, side: THREE.DoubleSide }),
            yellow: new THREE.MeshPhongMaterial({ color: 0xfab600, side: THREE.DoubleSide }),
            dark_gray: new THREE.MeshPhongMaterial({ color: 0x4d4d4d, side: THREE.DoubleSide })
        };
        
        this.init();
    }
    
    init() {
        console.log('EL-A3 3D Viewer: Initializing...');
        console.log('Container:', this.container);
        console.log('Container size:', this.container?.clientWidth, 'x', this.container?.clientHeight);
        
        // Scene setup
        this.scene = new THREE.Scene();
        this.scene.background = new THREE.Color(0x1a1a2e);
        
        // Camera setup - position for Z-up coordinate system
        const aspect = this.container.clientWidth / this.container.clientHeight;
        this.camera = new THREE.PerspectiveCamera(45, aspect, 0.001, 1000);
        this.camera.position.set(0.8, 0.8, 0.6);
        this.camera.up.set(0, 0, 1); // Z is up
        this.camera.lookAt(0, 0, 0.2);
        
        // Renderer setup
        this.renderer = new THREE.WebGLRenderer({ antialias: true });
        this.renderer.setSize(this.container.clientWidth, this.container.clientHeight);
        this.renderer.setPixelRatio(window.devicePixelRatio);
        this.renderer.shadowMap.enabled = true;
        this.container.appendChild(this.renderer.domElement);
        
        // Orbit controls
        this.controls = new OrbitControls(this.camera, this.renderer.domElement);
        this.controls.enableDamping = true;
        this.controls.dampingFactor = 0.05;
        this.controls.target.set(0, 0, 0.15);
        this.controls.update();
        
        // Lighting
        this.setupLighting();
        
        // Grid and axes
        this.setupGrid();
        
        // Load robot model
        this.loadRobot();
        
        // Window resize handler
        window.addEventListener('resize', () => this.onResize());
        
        // Control buttons
        this.setupControls();
        
        // State update listener
        window.addEventListener('robotStateUpdate', (e) => this.updateJoints(e.detail));
        
        // Animation loop
        this.animate();
        
        console.log('EL-A3 3D Viewer: Init complete');
    }
    
    hideLoading() {
        const loading = document.getElementById('viewerLoading');
        if (loading) {
            loading.style.display = 'none';
        }
        console.log('EL-A3 3D Viewer: All meshes loaded');
        
        // Debug: print scene info after loading
        this.debugSceneInfo();
        
        // Auto-fit camera to model
        this.fitCameraToModel();
    }
    
    updateLoadingProgress() {
        this.loadingCount++;
        const loading = document.getElementById('viewerLoading');
        if (loading) {
            const span = loading.querySelector('span');
            if (span) {
                span.textContent = `加载模型中... (${this.loadingCount}/${this.totalMeshes})`;
            }
        }
        if (this.loadingCount >= this.totalMeshes) {
            setTimeout(() => this.hideLoading(), 500);
        }
    }
    
    debugSceneInfo() {
        console.log('========== Scene Debug Info ==========');
        console.log('Scene children count:', this.scene.children.length);
        
        if (this.robot) {
            console.log('Robot group exists:', true);
            console.log('Robot children count:', this.robot.children.length);
            
            // Calculate bounding box
            const box = new THREE.Box3().setFromObject(this.robot);
            const size = new THREE.Vector3();
            const center = new THREE.Vector3();
            box.getSize(size);
            box.getCenter(center);
            
            console.log('Robot bounding box min:', box.min);
            console.log('Robot bounding box max:', box.max);
            console.log('Robot size:', size);
            console.log('Robot center:', center);
            console.log('Is bounding box empty:', box.isEmpty());
            
            // List all loaded meshes
            console.log('Loaded meshes:', this.loadedMeshes);
        } else {
            console.log('Robot group exists:', false);
        }
        
        console.log('Camera position:', this.camera.position);
        console.log('Camera target:', this.controls.target);
        console.log('=======================================');
    }
    
    fitCameraToModel() {
        if (!this.robot) return;
        
        const box = new THREE.Box3().setFromObject(this.robot);
        if (box.isEmpty()) {
            console.warn('Robot bounding box is empty, cannot fit camera');
            return;
        }
        
        const center = new THREE.Vector3();
        const size = new THREE.Vector3();
        box.getCenter(center);
        box.getSize(size);
        
        const maxDim = Math.max(size.x, size.y, size.z);
        const fov = this.camera.fov * (Math.PI / 180);
        let cameraDistance = Math.abs(maxDim / Math.sin(fov / 2)) * 1.5;
        
        // Ensure minimum distance
        cameraDistance = Math.max(cameraDistance, 0.5);
        
        // Position camera looking at center from an angle
        this.camera.position.set(
            center.x + cameraDistance * 0.7,
            center.y + cameraDistance * 0.7,
            center.z + cameraDistance * 0.5
        );
        this.camera.up.set(0, 0, 1);
        
        this.controls.target.copy(center);
        this.controls.update();
        
        console.log('Camera auto-fitted to model');
        console.log('Model center:', center);
        console.log('Model max dimension:', maxDim);
        console.log('Camera distance:', cameraDistance);
        console.log('New camera position:', this.camera.position);
    }
    
    setupLighting() {
        // Stronger ambient light
        const ambient = new THREE.AmbientLight(0xffffff, 0.8);
        this.scene.add(ambient);
        
        // Main directional light
        const directional = new THREE.DirectionalLight(0xffffff, 1.0);
        directional.position.set(5, 5, 10);
        directional.castShadow = true;
        this.scene.add(directional);
        
        // Fill lights from multiple angles
        const fill1 = new THREE.DirectionalLight(0x4080ff, 0.5);
        fill1.position.set(-5, -5, 5);
        this.scene.add(fill1);
        
        const fill2 = new THREE.DirectionalLight(0xffffff, 0.4);
        fill2.position.set(0, 0, -5);
        this.scene.add(fill2);
        
        const fill3 = new THREE.DirectionalLight(0xffffff, 0.3);
        fill3.position.set(5, -5, 3);
        this.scene.add(fill3);
    }
    
    setupGrid() {
        // Grid on XY plane (Z is up)
        this.gridHelper = new THREE.GridHelper(2, 40, 0x444466, 0x333344);
        this.gridHelper.rotation.x = Math.PI / 2; // Rotate to XY plane
        this.scene.add(this.gridHelper);
        
        // Axes helper (X=red, Y=green, Z=blue)
        const axesHelper = new THREE.AxesHelper(0.3);
        this.scene.add(axesHelper);
    }
    
    loadSTL(url, material, debugName = '') {
        return new Promise((resolve, reject) => {
            console.log(`Loading STL: ${debugName || url}`);
            this.stlLoader.load(
                url,
                (geometry) => {
                    // Compute original bounding box to check size
                    geometry.computeBoundingBox();
                    const originalBox = geometry.boundingBox.clone();
                    const originalSize = new THREE.Vector3();
                    originalBox.getSize(originalSize);
                    
                    console.log(`STL ${debugName} original size:`, originalSize);
                    
                    // Determine if scaling is needed
                    // If any dimension is > 1 (likely millimeters), scale down
                    const maxOriginalDim = Math.max(originalSize.x, originalSize.y, originalSize.z);
                    let scale = 1.0;
                    if (maxOriginalDim > 1.0) {
                        // STL is in millimeters, convert to meters
                        scale = 0.001;
                        console.log(`STL ${debugName} appears to be in mm, applying scale ${scale}`);
                    } else {
                        console.log(`STL ${debugName} appears to be in meters, no scaling needed`);
                    }
                    
                    geometry.scale(scale, scale, scale);
                    geometry.computeVertexNormals();
                    geometry.computeBoundingBox();
                    
                    const mesh = new THREE.Mesh(geometry, material);
                    mesh.castShadow = true;
                    mesh.receiveShadow = true;
                    mesh.name = debugName;
                    
                    // Log final size
                    const finalSize = new THREE.Vector3();
                    geometry.boundingBox.getSize(finalSize);
                    console.log(`STL ${debugName} final size:`, finalSize);
                    
                    this.loadedMeshes.push({
                        name: debugName,
                        originalSize: originalSize,
                        finalSize: finalSize,
                        scale: scale
                    });
                    
                    this.updateLoadingProgress();
                    resolve(mesh);
                },
                (progress) => {
                    // Progress callback
                },
                (error) => {
                    console.error(`Failed to load ${debugName || url}:`, error);
                    this.updateLoadingProgress();
                    resolve(null);
                }
            );
        });
    }
    
    async loadRobot() {
        console.log('EL-A3 3D Viewer: Loading robot model...');
        
        // Robot root group
        this.robot = new THREE.Group();
        this.robot.name = 'robot';
        this.scene.add(this.robot);
        
        // Total meshes count: Base(2) + Link1(3) + Link2(6) + Link3(2) + Link4(5) + Link5(1) + Link6(14) = 33
        this.totalMeshes = 33;
        
        // Build kinematic chain according to URDF
        await this.buildKinematicChain();
    }
    
    async buildKinematicChain() {
        const basePath = '/urdf/meshes/';
        
        // ========== Base Link (l1_urdf_urdf_asm) ==========
        // This is fixed to the world
        const baseLink = new THREE.Group();
        baseLink.name = 'base_link';
        this.robot.add(baseLink);
        
        // Load base meshes (origin: rpy="0 0 0" xyz="0 0 0")
        const baseMesh1 = await this.loadSTL(basePath + 'rs00_l1.stl', this.materials.orange, 'base_rs00_l1');
        if (baseMesh1) baseLink.add(baseMesh1);
        
        const baseMesh2 = await this.loadSTL(basePath + 'l1_base.stl', this.materials.gray, 'base_l1_base');
        if (baseMesh2) baseLink.add(baseMesh2);
        
        // ========== L1_joint ==========
        // origin: xyz="0 0 0.0054" rpy="0 0 0"
        // axis: xyz="0 0 1" (rotate around Z)
        const joint1 = new THREE.Group();
        joint1.name = 'L1_joint';
        joint1.position.set(0, 0, 0.0054);
        baseLink.add(joint1);
        this.jointGroups['L1_joint'] = joint1;
        
        // Link1 meshes (l1_link_urdf_asm)
        // visual origin: xyz="0 0 -0.0054"
        const link1Mesh1 = await this.loadSTL(basePath + 'l1_link_stop.stl', this.materials.light_blue, 'link1_stop');
        if (link1Mesh1) {
            link1Mesh1.position.set(0, 0, -0.0054);
            joint1.add(link1Mesh1);
        }
        
        const link1Mesh2 = await this.loadSTL(basePath + 'l1_link_drv.stl', this.materials.light_blue, 'link1_drv');
        if (link1Mesh2) {
            link1Mesh2.position.set(0, 0, -0.0054);
            joint1.add(link1Mesh2);
        }
        
        const link1Mesh3 = await this.loadSTL(basePath + 'l2_stop.stl', this.materials.orange, 'link1_l2stop');
        if (link1Mesh3) {
            link1Mesh3.position.set(0, 0, -0.0054);
            joint1.add(link1Mesh3);
        }
        
        // ========== L2_joint ==========
        // origin: xyz="-0.0176846 0 0.0606" rpy="1.5708 0 0"
        // axis: xyz="0 0 1"
        const joint2Parent = new THREE.Group();
        joint2Parent.name = 'L2_joint_parent';
        joint2Parent.position.set(-0.0176846, 0, 0.0606);
        joint2Parent.rotation.set(Math.PI/2, 0, 0, 'XYZ');
        joint1.add(joint2Parent);
        
        const joint2 = new THREE.Group();
        joint2.name = 'L2_joint';
        joint2Parent.add(joint2);
        this.jointGroups['L2_joint'] = joint2;
        
        // Link2 meshes (l2_l3_urdf_asm) - 6 meshes
        // Container for meshes with origin: rpy="-1.5708 0 0" xyz="0.0176846 -0.066 0"
        const link2Container = new THREE.Group();
        link2Container.name = 'link2_container';
        link2Container.position.set(0.0176846, -0.066, 0);
        link2Container.rotation.set(-Math.PI/2, 0, 0, 'XYZ');
        joint2.add(link2Container);
        
        // mainlink_back.stl - origin offset z=0.0005
        const link2Mesh1 = await this.loadSTL(basePath + 'mainlink_back.stl', this.materials.gray, 'link2_mainlink');
        if (link2Mesh1) {
            link2Mesh1.position.set(0, 0, 0.0005);
            link2Container.add(link2Mesh1);
        }
        
        // 6705.stl (white) - origin offset z=0.0005
        const link2Mesh2 = await this.loadSTL(basePath + '6705.stl', this.materials.white, 'link2_6705');
        if (link2Mesh2) {
            link2Mesh2.position.set(0, 0, 0.0005);
            link2Container.add(link2Mesh2);
        }
        
        // rs00_l2.stl (orange)
        const link2Mesh3 = await this.loadSTL(basePath + 'rs00_l2.stl', this.materials.orange, 'link2_rs00_l2');
        if (link2Mesh3) link2Container.add(link2Mesh3);
        
        // mainlink_motor.stl (gray)
        const link2Mesh4 = await this.loadSTL(basePath + 'mainlink_motor.stl', this.materials.gray, 'link2_motor');
        if (link2Mesh4) link2Container.add(link2Mesh4);
        
        // 6705__2.stl (blue) - origin offset z=0.0005
        const link2Mesh5 = await this.loadSTL(basePath + '6705__2.stl', this.materials.blue, 'link2_6705_2');
        if (link2Mesh5) {
            link2Mesh5.position.set(0, 0, 0.0005);
            link2Container.add(link2Mesh5);
        }
        
        // rs00_l3.stl (orange)
        const link2Mesh6 = await this.loadSTL(basePath + 'rs00_l3.stl', this.materials.orange, 'link2_rs00_l3');
        if (link2Mesh6) link2Container.add(link2Mesh6);
        
        // ========== L3_joint ==========
        // origin: xyz="0.19 0 0" rpy="0 0 0"
        // axis: xyz="0 0 1"
        const joint3 = new THREE.Group();
        joint3.name = 'L3_joint';
        joint3.position.set(0.19, 0, 0);
        joint2.add(joint3);
        this.jointGroups['L3_joint'] = joint3;
        
        // Link3 meshes (l3_lnik_urdf_asm) - 2 meshes
        // visual origin: rpy="-1.5708 0 0" xyz="-0.172315 -0.066 0"
        const link3Container = new THREE.Group();
        link3Container.name = 'link3_container';
        link3Container.position.set(-0.172315, -0.066, 0);
        link3Container.rotation.set(-Math.PI/2, 0, 0, 'XYZ');
        joint3.add(link3Container);
        
        const link3Mesh1 = await this.loadSTL(basePath + 'l4_link_back.stl', this.materials.dark_gray, 'link3_back');
        if (link3Mesh1) link3Container.add(link3Mesh1);
        
        const link3Mesh2 = await this.loadSTL(basePath + 'l4_link_drv.stl', this.materials.dark_gray, 'link3_drv');
        if (link3Mesh2) link3Container.add(link3Mesh2);
        
        // ========== L4_joint ==========
        // origin: xyz="-0.15 0.06 0" rpy="0 0 0"
        // axis: xyz="0 0 1"
        const joint4 = new THREE.Group();
        joint4.name = 'L4_joint';
        joint4.position.set(-0.15, 0.06, 0);
        joint3.add(joint4);
        this.jointGroups['L4_joint'] = joint4;
        
        // Link4 meshes (l4_l5_urdf_asm) - 5 meshes
        // visual origin: rpy="-1.5708 0 0" xyz="-0.0223154 -0.126 0"
        const link4Container = new THREE.Group();
        link4Container.name = 'link4_container';
        link4Container.position.set(-0.0223154, -0.126, 0);
        link4Container.rotation.set(-Math.PI/2, 0, 0, 'XYZ');
        joint4.add(link4Container);
        
        // el05_l5.stl (white)
        const link4Mesh1 = await this.loadSTL(basePath + 'el05_l5.stl', this.materials.white, 'link4_el05_l5');
        if (link4Mesh1) link4Container.add(link4Mesh1);
        
        // l4_l5_link_back.stl (white)
        const link4Mesh2 = await this.loadSTL(basePath + 'l4_l5_link_back.stl', this.materials.white, 'link4_back');
        if (link4Mesh2) link4Container.add(link4Mesh2);
        
        // l5_shake.stl (light_blue)
        const link4Mesh3 = await this.loadSTL(basePath + 'l5_shake.stl', this.materials.light_blue, 'link4_shake');
        if (link4Mesh3) link4Container.add(link4Mesh3);
        
        // l4_l5_link_back__2.stl (orange)
        const link4Mesh4 = await this.loadSTL(basePath + 'l4_l5_link_back__2.stl', this.materials.orange, 'link4_back_2');
        if (link4Mesh4) link4Container.add(link4Mesh4);
        
        // el05_l4.stl (white)
        const link4Mesh5 = await this.loadSTL(basePath + 'el05_l4.stl', this.materials.white, 'link4_el05_l4');
        if (link4Mesh5) link4Container.add(link4Mesh5);
        
        // ========== L5_joint ==========
        // origin: xyz="-0.0492 0.038 0" rpy="1.5708 0 0"
        // axis: xyz="0 0 1"
        const joint5Parent = new THREE.Group();
        joint5Parent.name = 'L5_joint_parent';
        joint5Parent.position.set(-0.0492, 0.038, 0);
        joint5Parent.rotation.set(Math.PI/2, 0, 0, 'XYZ');
        joint4.add(joint5Parent);
        
        const joint5 = new THREE.Group();
        joint5.name = 'L5_joint';
        joint5Parent.add(joint5);
        this.jointGroups['L5_joint'] = joint5;
        
        // Link5 mesh (part_9) - 1 mesh
        // visual origin: rpy="3.14159 0 0" xyz="0.0268846 0 0.164"
        const link5Mesh = await this.loadSTL(basePath + 'part_9.stl', this.materials.orange, 'link5_part9');
        if (link5Mesh) {
            link5Mesh.position.set(0.0268846, 0, 0.164);
            link5Mesh.rotation.set(Math.PI, 0, 0, 'XYZ');
            joint5.add(link5Mesh);
        }
        
        // ========== L6_joint ==========
        // origin: xyz="0.00805 0 0.038" rpy="1.5708 0 -1.5708"
        // axis: xyz="0 0 1"
        const joint6Parent = new THREE.Group();
        joint6Parent.name = 'L6_joint_parent';
        joint6Parent.position.set(0.00805, 0, 0.038);
        // rpy order is roll(X), pitch(Y), yaw(Z) -> apply in ZYX order for extrinsic
        joint6Parent.rotation.set(Math.PI/2, 0, -Math.PI/2, 'ZYX');
        joint5.add(joint6Parent);
        
        const joint6 = new THREE.Group();
        joint6.name = 'L6_joint';
        joint6Parent.add(joint6);
        this.jointGroups['L6_joint'] = joint6;
        
        // Link6 meshes (l5_l6_urdf_asm - end effector) - 14 meshes
        
        // part_2.stl (gray) - origin: rpy="0 1.5708 0" xyz="0 0 0.062"
        const eeMesh1 = await this.loadSTL(basePath + 'part_2.stl', this.materials.gray, 'ee_part2');
        if (eeMesh1) {
            eeMesh1.position.set(0, 0, 0.062);
            eeMesh1.rotation.set(0, Math.PI/2, 0, 'XYZ');
            joint6.add(eeMesh1);
        }
        
        // hand_r.stl (blue) - origin: xyz="0 0 0.0755"
        const eeMesh2 = await this.loadSTL(basePath + 'hand_r.stl', this.materials.blue, 'ee_hand');
        if (eeMesh2) {
            eeMesh2.position.set(0, 0, 0.0755);
            joint6.add(eeMesh2);
        }
        
        // el05.stl (white) - origin: xyz="0 0 0.0755"
        const eeMesh3 = await this.loadSTL(basePath + 'el05.stl', this.materials.white, 'ee_el05_1');
        if (eeMesh3) {
            eeMesh3.position.set(0, 0, 0.0755);
            joint6.add(eeMesh3);
        }
        
        // rack__30_teeth.stl (white) - origin: xyz="0 0 0.0755"
        const eeMesh4 = await this.loadSTL(basePath + 'rack__30_teeth.stl', this.materials.white, 'ee_rack1');
        if (eeMesh4) {
            eeMesh4.position.set(0, 0, 0.0755);
            joint6.add(eeMesh4);
        }
        
        // manifold_solid_brep_39_1.stl (yellow) - origin: xyz="0.003 0 0.0755"
        const eeMesh5 = await this.loadSTL(basePath + 'manifold_solid_brep_39_1.stl', this.materials.yellow, 'ee_brep39_1');
        if (eeMesh5) {
            eeMesh5.position.set(0.003, 0, 0.0755);
            joint6.add(eeMesh5);
        }
        
        // rack__30_teeth__2.stl (white) - origin: xyz="0 0 0.0755"
        const eeMesh6 = await this.loadSTL(basePath + 'rack__30_teeth__2.stl', this.materials.white, 'ee_rack2');
        if (eeMesh6) {
            eeMesh6.position.set(0, 0, 0.0755);
            joint6.add(eeMesh6);
        }
        
        // manifold_solid_brep_39_1__2.stl (yellow) - origin: xyz="-0.003 0 0.0755"
        const eeMesh7 = await this.loadSTL(basePath + 'manifold_solid_brep_39_1__2.stl', this.materials.yellow, 'ee_brep39_2');
        if (eeMesh7) {
            eeMesh7.position.set(-0.003, 0, 0.0755);
            joint6.add(eeMesh7);
        }
        
        // rack_base.stl (light_blue) - origin: xyz="0 0 0.0755"
        const eeMesh8 = await this.loadSTL(basePath + 'rack_base.stl', this.materials.light_blue, 'ee_rack_base');
        if (eeMesh8) {
            eeMesh8.position.set(0, 0, 0.0755);
            joint6.add(eeMesh8);
        }
        
        // rack_str.stl (light_blue) - origin: xyz="0 0 0.0755"
        const eeMesh9 = await this.loadSTL(basePath + 'rack_str.stl', this.materials.light_blue, 'ee_rack_str');
        if (eeMesh9) {
            eeMesh9.position.set(0, 0, 0.0755);
            joint6.add(eeMesh9);
        }
        
        // part_1.stl (light_blue) - origin: rpy="0 1.5708 0" xyz="0 0 0.062"
        const eeMesh10 = await this.loadSTL(basePath + 'part_1.stl', this.materials.light_blue, 'ee_part1');
        if (eeMesh10) {
            eeMesh10.position.set(0, 0, 0.062);
            eeMesh10.rotation.set(0, Math.PI/2, 0, 'XYZ');
            joint6.add(eeMesh10);
        }
        
        // el05.stl (white) second instance - origin: rpy="3.14159 0 0" xyz="0 0 0.0305"
        const eeMesh11 = await this.loadSTL(basePath + 'el05.stl', this.materials.white, 'ee_el05_2');
        if (eeMesh11) {
            eeMesh11.position.set(0, 0, 0.0305);
            eeMesh11.rotation.set(Math.PI, 0, 0, 'XYZ');
            joint6.add(eeMesh11);
        }
        
        // spur_gear__14_teeth.stl (white) - origin: xyz="0 0 0.0755"
        const eeMesh12 = await this.loadSTL(basePath + 'spur_gear__14_teeth.stl', this.materials.white, 'ee_gear');
        if (eeMesh12) {
            eeMesh12.position.set(0, 0, 0.0755);
            joint6.add(eeMesh12);
        }
        
        // part_15.stl (blue) - origin: xyz="0 0 0.0755"
        const eeMesh13 = await this.loadSTL(basePath + 'part_15.stl', this.materials.blue, 'ee_part15');
        if (eeMesh13) {
            eeMesh13.position.set(0, 0, 0.0755);
            joint6.add(eeMesh13);
        }
        
        // manifold_solid_brep_40_1.stl (gray) - origin: xyz="0 0 0.0755"
        const eeMesh14 = await this.loadSTL(basePath + 'manifold_solid_brep_40_1.stl', this.materials.gray, 'ee_brep40');
        if (eeMesh14) {
            eeMesh14.position.set(0, 0, 0.0755);
            joint6.add(eeMesh14);
        }
        
        console.log('Robot kinematic chain built successfully');
        console.log('Joint groups:', Object.keys(this.jointGroups));
    }
    
    updateJoints(state) {
        if (!state || !state.joints) return;
        
        state.joints.forEach((joint) => {
            const jointGroup = this.jointGroups[joint.name];
            if (jointGroup) {
                // All joints rotate around local Z axis
                jointGroup.rotation.z = joint.position;
            }
        });
    }
    
    setupControls() {
        // Reset view button
        const resetBtn = document.getElementById('resetViewBtn');
        if (resetBtn) {
            resetBtn.addEventListener('click', () => {
                this.fitCameraToModel();
            });
        }
        
        // Toggle grid button
        const gridBtn = document.getElementById('toggleGridBtn');
        if (gridBtn) {
            gridBtn.addEventListener('click', () => {
                this.showGrid = !this.showGrid;
                if (this.gridHelper) {
                    this.gridHelper.visible = this.showGrid;
                }
            });
        }
    }
    
    onResize() {
        const width = this.container.clientWidth;
        const height = this.container.clientHeight;
        
        this.camera.aspect = width / height;
        this.camera.updateProjectionMatrix();
        this.renderer.setSize(width, height);
    }
    
    animate() {
        requestAnimationFrame(() => this.animate());
        
        this.controls.update();
        this.renderer.render(this.scene, this.camera);
    }
}

// Initialize viewer when DOM is ready
let robotViewer = null;

if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', () => {
        console.log('EL-A3 3D Viewer: DOM loaded, creating viewer...');
        robotViewer = new RobotViewer('viewer3d');
        window.robotViewer = robotViewer;
    });
} else {
    console.log('EL-A3 3D Viewer: DOM already ready, creating viewer...');
    robotViewer = new RobotViewer('viewer3d');
    window.robotViewer = robotViewer;
}

export { RobotViewer };
