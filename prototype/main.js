import * as THREE from 'three'
import { OrbitControls } from 'three/addons/controls/OrbitControls.js'
import { PLYLoader }     from 'three/addons/loaders/PLYLoader.js'

// ── Renderer ─────────────────────────────────────────────────────────
const renderer = new THREE.WebGLRenderer({ antialias: true })
renderer.setPixelRatio(Math.min(devicePixelRatio, 2))
renderer.setSize(innerWidth, innerHeight)
renderer.setClearColor(0x07080a)
renderer.outputColorSpace = THREE.SRGBColorSpace
renderer.toneMapping      = THREE.NoToneMapping
document.body.appendChild(renderer.domElement)

// ── Scene / Camera ───────────────────────────────────────────────────
const scene  = new THREE.Scene()
const camera = new THREE.PerspectiveCamera(55, innerWidth / innerHeight, 1, 300000)
camera.up.set(0, 0, 1)   // Z-up matches the LiDAR scene coordinate system

// ── Controls ─────────────────────────────────────────────────────────
const controls = new OrbitControls(camera, renderer.domElement)
controls.enableDamping = true
controls.dampingFactor  = 0.05
controls.minDistance    = 10
controls.maxDistance    = 50000

// ── Materials ─────────────────────────────────────────────────────────
let ptSize = 2.0   // pixels

const matVertex = new THREE.PointsMaterial({
  size: ptSize,
  vertexColors: true,
  sizeAttenuation: false,
  toneMapped: false,
})

const matElevation = new THREE.ShaderMaterial({
  uniforms: {
    minZ: { value: 0 },
    maxZ: { value: 800 },
    ptSz: { value: ptSize },
  },
  vertexShader: `
    uniform float minZ;
    uniform float maxZ;
    uniform float ptSz;
    varying float vT;
    void main() {
      vT = clamp((position.z - minZ) / (maxZ - minZ), 0.0, 1.0);
      gl_PointSize = ptSz;
      gl_Position  = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
    }
  `,
  fragmentShader: `
    varying float vT;
    void main() {
      vec3 c0 = vec3(0.267, 0.005, 0.329);
      vec3 c1 = vec3(0.128, 0.563, 0.551);
      vec3 c2 = vec3(0.993, 0.906, 0.144);
      vec3 col = vT < 0.5
        ? mix(c0, c1, vT * 2.0)
        : mix(c1, c2, (vT - 0.5) * 2.0);
      gl_FragColor = vec4(col, 1.0);
    }
  `,
})
matElevation.toneMapped = false

// ── Load PLY ──────────────────────────────────────────────────────────
// PLY has x,y,z (float32) + red,green,blue (uint8) — PLYLoader maps
// uchar colors to geometry.attributes.color (Float32, 0-1)
let cloud    = null
let useVertex = true

// Scene selectable via ?scene=<id> (file = ./pointclouds/<id>.ply). Default: Dibona test.
const sceneId = new URLSearchParams(location.search).get('scene') || 'dibona-025-lit'

const loader = new PLYLoader()
loader.load(
  `./pointclouds/${sceneId}.ply`,
  (geometry) => {
    geometry.computeBoundingBox()
    const box    = geometry.boundingBox
    const center = new THREE.Vector3()
    box.getCenter(center)
    const size   = new THREE.Vector3()
    box.getSize(size)

    // PLY vertex colors are sRGB — tell Three.js so it linearises before use
    // then re-encodes at output via outputColorSpace=SRGBColorSpace (net = identity)
    if (geometry.attributes.color) {
      geometry.attributes.color.colorSpace = THREE.SRGBColorSpace
    }

    matElevation.uniforms.minZ.value = box.min.z
    matElevation.uniforms.maxZ.value = box.max.z

    const dist = size.length() * 0.75
    controls.target.set(0, 0, 0)  // PLY centered on OrbitTarget
    camera.position.set(0, -dist, dist * 0.55)
    controls.update()

    // Store initial state for reset
    const initPos    = camera.position.clone()
    const initTarget = new THREE.Vector3(0, 0, 0)
    document.getElementById('reset-btn').addEventListener('click', () => {
      camera.position.copy(initPos)
      controls.target.copy(initTarget)
      controls.update()
    })

    cloud = new THREE.Points(geometry, matVertex)
    scene.add(cloud)

    const npts = geometry.attributes.position.count
    document.getElementById('pts-line').textContent =
      `${(npts / 1e6).toFixed(2)}M pts  ·  ${(size.x / 1000).toFixed(2)} km`

    document.getElementById('loading').classList.add('hidden')
    document.getElementById('ui').classList.add('visible')
    document.getElementById('telemetry').classList.add('visible')
  },
  (xhr) => {
    if (xhr.total) {
      const pct = Math.round(xhr.loaded / xhr.total * 100)
      document.getElementById('loading').textContent = `Loading… ${pct}%`
    }
  },
  (err) => {
    console.error('PLYLoader error:', err)
    document.getElementById('loading').textContent = 'Load error — check console'
  }
)

// ── UI ────────────────────────────────────────────────────────────────
document.getElementById('size-slider').addEventListener('input', e => {
  ptSize = parseFloat(e.target.value)
  matVertex.size = ptSize
  matElevation.uniforms.ptSz.value = ptSize
})

document.getElementById('color-btn').addEventListener('click', () => {
  if (!cloud) return
  useVertex = !useVertex
  cloud.material = useVertex ? matVertex : matElevation
  document.getElementById('color-btn').textContent =
    useVertex ? '⬡ Elevation mode' : '⬡ Satellite + light'
})

// ── FPS ───────────────────────────────────────────────────────────────
let frames = 0, fpsT = performance.now()
const fpsEl = document.getElementById('fps-line')

// ── Resize ───────────────────────────────────────────────────────────
window.addEventListener('resize', () => {
  camera.aspect = innerWidth / innerHeight
  camera.updateProjectionMatrix()
  renderer.setSize(innerWidth, innerHeight)
})

// ── Render loop ───────────────────────────────────────────────────────
;(function animate() {
  requestAnimationFrame(animate)
  controls.update()
  renderer.render(scene, camera)

  frames++
  const now = performance.now()
  if (now - fpsT >= 1000) {
    fpsEl.textContent = `${frames} fps`
    frames = 0
    fpsT   = now
  }
})()
