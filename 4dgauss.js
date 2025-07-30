
let PATH = [];
function syncFromView () {
  // pull yaw / pitch / position out of *current* viewMatrix
  const [y, p, t] = getYawPitchPosFromView(viewMatrix);

  yaw     = y;
  pitch   = p;
  camPos  = [...t];

  camCur  = { pos:[...t], yaw:y, pitch:p };
  camDst  = { pos:[...t], yaw:y, pitch:p };


  tBlend  = 1;              // cancel any in-flight interpolation
}



const wrapPi = a => ((a + Math.PI) % (2 * Math.PI)) - Math.PI;
const BLEND_MS = 1000;

const DIAG_FOV_DEG = 70;                     // try 65-75 for a “normal” lens
const DIAG_FOV     = DIAG_FOV_DEG * Math.PI/180;

let fx = 1, fy = 1;                          // updated every resize()

function updateFocalLengths () {
  const minDim = Math.min(innerWidth, innerHeight);   // << key line
  const f      = 0.5 * minDim / Math.tan(DIAG_FOV/2);

  fx = f;                // square-pixel focal lengths
  fy = f;
}

function forward(rot3) {            // camera –Z in world space
  const r = rot3.flat();
  return [-r[2], -r[5], -r[8]];
}

function matT(m){ return [[m[0][0],m[1][0],m[2][0]],
                          [m[0][1],m[1][1],m[2][1]],
                          [m[0][2],m[1][2],m[2][2]]]; }
function mul3(A,B){
  return A.map((_,r)=>[0,1,2].map(c=>A[r][0]*B[0][c]+A[r][1]*B[1][c]+A[r][2]*B[2][c]));
}

function matToEulerYxz(R){
  const d = 180/Math.PI;
  const pitch = Math.asin(Math.max(-1,Math.min( 1,-R[1][2])));        // X
  const yaw   = Math.atan2( R[0][2],  R[2][2]);                       // Y
  const roll  = Math.atan2( R[1][0],  R[1][1]);                       // Z
  return [yaw*d, pitch*d, roll*d];
}

function updateYawPitchFromView () {
  const inv = invert4(viewMatrix);   // camera → world (row-major)

  // BACK vector components (row 2 of the matrix)
  const bx = inv[8];
  const by = inv[9];
  const bz = inv[10];

  // heading (yaw) and elevation (pitch) in radians
  yaw   = Math.atan2(bx, bz);  // yaw:   atan2( BACK.x , BACK.z )
  pitch = Math.asin(-by);      // pitch: asin( -BACK.y )

  // keep world-space position in sync
  camPos = [inv[12], inv[13], inv[14]];
}

function safeAsin(x){ return Math.asin(Math.max(-1, Math.min(1, x))); }

function getYawPitchPosFromView(viewMatrix){
  const inv = invert4(viewMatrix);
  const bx = inv[8],  by = inv[9],  bz = inv[10];

  const yaw   = Math.atan2(bx, bz);   // unchanged
  const pitch = safeAsin(-by);        // ← clamp first
  return [yaw, pitch, [inv[12], inv[13], inv[14]]];
}

function makeViewMatrix(yaw, pitch, pos) {
  const cy = Math.cos(yaw),  sy = Math.sin(yaw);
  const cp = Math.cos(pitch), sp = Math.sin(pitch);

  /* camera-to-world 3×3 (row-major) — no roll term anywhere */
  const right = [  cy, 0, -sy ];
  const up    = [  sy*sp,  cp,  cy*sp ];
  const back  = [  sy*cp, -sp,  cy*cp ];   // forward = –back

  /* build 4×4 camera→world, then invert → view */
  return invert4([
    right[0], right[1], right[2], 0,
    up[0],    up[1],    up[2],    0,
    back[0],  back[1],  back[2],  0,
    pos[0],   pos[1],   pos[2],   1,
  ]);
}

function yawPitchFromRot3 (R) {
  const bx = R[0][2];              // BACK.x
  const by = R[1][2];              // BACK.y
  const bz = R[2][2];              // BACK.z
  return [ Math.atan2(bx, bz),     // yaw
           Math.asin(-by) ];       // pitch
}
function matrixFromYawPitch(yaw,pitch){  // IDENTICAL basis the drag code builds
  const cy = Math.cos(yaw),  sy = Math.sin(yaw);
  const cp = Math.cos(pitch),sp = Math.sin(pitch);
  return [
    [  cy,        0,   -sy      ],   // right (X)
    [  sy*sp,     cp,   cy*sp   ],   // up    (Y)
    [  sy*cp,    -sp,   cy*cp   ],   // back  (Z)
  ];
}



function getProjectionMatrix(fx, fy, width, height) {
    const znear = 0.2;
    const zfar = 200;
    return [
        [(2 * fx) / width, 0, 0, 0],
        [0, -(2 * fy) / height, 0, 0],
        [0, 0, zfar / (zfar - znear), 1],
        [0, 0, -(zfar * znear) / (zfar - znear), 0],
    ].flat();
}

function getViewMatrix(camera) {
    const R = camera.rotation.flat();
    const t = camera.position;
    const camToWorld = [
        [R[0], R[1], R[2], 0],
        [R[3], R[4], R[5], 0],
        [R[6], R[7], R[8], 0],
        [
            -t[0] * R[0] - t[1] * R[3] - t[2] * R[6],
            -t[0] * R[1] - t[1] * R[4] - t[2] * R[7],
            -t[0] * R[2] - t[1] * R[5] - t[2] * R[8],
            1,
        ],
    ].flat();
    return camToWorld;
}

function multiply4(a, b) {
    return [
        b[0] * a[0] + b[1] * a[4] + b[2] * a[8] + b[3] * a[12],
        b[0] * a[1] + b[1] * a[5] + b[2] * a[9] + b[3] * a[13],
        b[0] * a[2] + b[1] * a[6] + b[2] * a[10] + b[3] * a[14],
        b[0] * a[3] + b[1] * a[7] + b[2] * a[11] + b[3] * a[15],
        b[4] * a[0] + b[5] * a[4] + b[6] * a[8] + b[7] * a[12],
        b[4] * a[1] + b[5] * a[5] + b[6] * a[9] + b[7] * a[13],
        b[4] * a[2] + b[5] * a[6] + b[6] * a[10] + b[7] * a[14],
        b[4] * a[3] + b[5] * a[7] + b[6] * a[11] + b[7] * a[15],
        b[8] * a[0] + b[9] * a[4] + b[10] * a[8] + b[11] * a[12],
        b[8] * a[1] + b[9] * a[5] + b[10] * a[9] + b[11] * a[13],
        b[8] * a[2] + b[9] * a[6] + b[10] * a[10] + b[11] * a[14],
        b[8] * a[3] + b[9] * a[7] + b[10] * a[11] + b[11] * a[15],
        b[12] * a[0] + b[13] * a[4] + b[14] * a[8] + b[15] * a[12],
        b[12] * a[1] + b[13] * a[5] + b[14] * a[9] + b[15] * a[13],
        b[12] * a[2] + b[13] * a[6] + b[14] * a[10] + b[15] * a[14],
        b[12] * a[3] + b[13] * a[7] + b[14] * a[11] + b[15] * a[15],
    ];
}

function invert4(a) {
    let b00 = a[0] * a[5] - a[1] * a[4];
    let b01 = a[0] * a[6] - a[2] * a[4];
    let b02 = a[0] * a[7] - a[3] * a[4];
    let b03 = a[1] * a[6] - a[2] * a[5];
    let b04 = a[1] * a[7] - a[3] * a[5];
    let b05 = a[2] * a[7] - a[3] * a[6];
    let b06 = a[8] * a[13] - a[9] * a[12];
    let b07 = a[8] * a[14] - a[10] * a[12];
    let b08 = a[8] * a[15] - a[11] * a[12];
    let b09 = a[9] * a[14] - a[10] * a[13];
    let b10 = a[9] * a[15] - a[11] * a[13];
    let b11 = a[10] * a[15] - a[11] * a[14];
    let det =
        b00 * b11 - b01 * b10 + b02 * b09 + b03 * b08 - b04 * b07 + b05 * b06;
    if (!det) return null;
    return [
        (a[5] * b11 - a[6] * b10 + a[7] * b09) / det,
        (a[2] * b10 - a[1] * b11 - a[3] * b09) / det,
        (a[13] * b05 - a[14] * b04 + a[15] * b03) / det,
        (a[10] * b04 - a[9] * b05 - a[11] * b03) / det,
        (a[6] * b08 - a[4] * b11 - a[7] * b07) / det,
        (a[0] * b11 - a[2] * b08 + a[3] * b07) / det,
        (a[14] * b02 - a[12] * b05 - a[15] * b01) / det,
        (a[8] * b05 - a[10] * b02 + a[11] * b01) / det,
        (a[4] * b10 - a[5] * b08 + a[7] * b06) / det,
        (a[1] * b08 - a[0] * b10 - a[3] * b06) / det,
        (a[12] * b04 - a[13] * b02 + a[15] * b00) / det,
        (a[9] * b02 - a[8] * b04 - a[11] * b00) / det,
        (a[5] * b07 - a[4] * b09 - a[6] * b06) / det,
        (a[0] * b09 - a[1] * b07 + a[2] * b06) / det,
        (a[13] * b01 - a[12] * b03 - a[14] * b00) / det,
        (a[8] * b03 - a[9] * b01 + a[10] * b00) / det,
    ];
}

function rotate4(a, rad, x, y, z) {
    let len = Math.hypot(x, y, z);
    x /= len;
    y /= len;
    z /= len;
    let s = Math.sin(rad);
    let c = Math.cos(rad);
    let t = 1 - c;
    let b00 = x * x * t + c;
    let b01 = y * x * t + z * s;
    let b02 = z * x * t - y * s;
    let b10 = x * y * t - z * s;
    let b11 = y * y * t + c;
    let b12 = z * y * t + x * s;
    let b20 = x * z * t + y * s;
    let b21 = y * z * t - x * s;
    let b22 = z * z * t + c;
    return [
        a[0] * b00 + a[4] * b01 + a[8] * b02,
        a[1] * b00 + a[5] * b01 + a[9] * b02,
        a[2] * b00 + a[6] * b01 + a[10] * b02,
        a[3] * b00 + a[7] * b01 + a[11] * b02,
        a[0] * b10 + a[4] * b11 + a[8] * b12,
        a[1] * b10 + a[5] * b11 + a[9] * b12,
        a[2] * b10 + a[6] * b11 + a[10] * b12,
        a[3] * b10 + a[7] * b11 + a[11] * b12,
        a[0] * b20 + a[4] * b21 + a[8] * b22,
        a[1] * b20 + a[5] * b21 + a[9] * b22,
        a[2] * b20 + a[6] * b21 + a[10] * b22,
        a[3] * b20 + a[7] * b21 + a[11] * b22,
        ...a.slice(12, 16),
    ];
}

function translate4(a, x, y, z) {
    return [
        ...a.slice(0, 12),
        a[0] * x + a[4] * y + a[8] * z + a[12],
        a[1] * x + a[5] * y + a[9] * z + a[13],
        a[2] * x + a[6] * y + a[10] * z + a[14],
        a[3] * x + a[7] * y + a[11] * z + a[15],
    ];
}

function createWorker(self) {
    let motion = null;
    let motionDict = null;
    let buffer;
    let vertexCount = 0;
    let viewProj;
    // 6*4 + 4 + 4 = 8*4
    // XYZ - Position (Float32)
    // XYZ - Scale (Float32)
    // RGBA - colors (uint8)
    // IJKL - quaternion/rot (uint8)
    const rowLength = 3 * 4 + 3 * 4 + 4 + 4;
    let lastProj = [];
    let depthIndex = new Uint32Array();
    let lastVertexCount = 0;

    var _floatView = new Float32Array(1);
    var _int32View = new Int32Array(_floatView.buffer);

    const f16 = (() => {
        const f32 = new Float32Array(1);
        const u32 = new Uint32Array(f32.buffer);
        return h => {
            const s =  (h & 0x8000) << 16;
            let   e = (h & 0x7C00) >> 10;
            let   f =  h & 0x03FF;
            if (!e) {                        // sub‑normal / zero
            if (!f) { u32[0] = s; return f32[0]; }
            e = 1; while (!(f & 0x400)) { f <<= 1; --e; }
            f &= 0x3FF;
            } else if (e === 31) {           // Inf / NaN
            u32[0] = s | 0x7F800000 | (f << 13); return f32[0];
            }
            e = e + (127 - 15);
            u32[0] = s | (e << 23) | (f << 13);
            return f32[0];
        };
        })();

        // ---------------------------------------------------------------------------
        //  Decode one splat (8 × uint32) -> plain object
        function decodeMotionRow(u32, idx) {
        const base = idx * 8;
        const lo = word => word & 0xFFFF;
        const hi = word => word >>> 16;

        const w = n => u32[base + n];        // convenience alias

        return {
            c1:    [ f16(lo(w(0))), f16(hi(w(0))), f16(lo(w(1))) ],
            c2:    [ f16(hi(w(1))), f16(lo(w(2))), f16(hi(w(2))) ],
            c3:    [ f16(lo(w(3))), f16(hi(w(3))), f16(lo(w(4))) ],
            omega: [ f16(lo(w(5))), f16(hi(w(5))), f16(lo(w(6))) ],
            mu:     f16(lo(w(7))),
            sigma:  f16(hi(w(7))),
        };
        }

    function floatToHalf(float) {
        _floatView[0] = float;
        var f = _int32View[0];

        var sign = (f >> 31) & 0x0001;
        var exp = (f >> 23) & 0x00ff;
        var frac = f & 0x007fffff;

        var newExp;
        if (exp == 0) {
            newExp = 0;
        } else if (exp < 113) {
            newExp = 0;
            frac |= 0x00800000;
            frac = frac >> (113 - exp);
            if (frac & 0x01000000) {
                newExp = 1;
                frac = 0;
            }
        } else if (exp < 142) {
            newExp = exp - 112;
        } else {
            newExp = 31;
            frac = 0;
        }

        return (sign << 15) | (newExp << 10) | (frac >> 13);
    }

    function packHalf2x16(x, y) {
        return (floatToHalf(x) | (floatToHalf(y) << 16)) >>> 0;
    }
    function halfToFloat(h){                   // IEEE‑754 half → float32
        const s=(h>>15)&1, e=(h>>10)&31, f=h&1023;
        if(!e)      return (s?-1:1)*Math.pow(2,-14)*(f/1024);
        if(e===31)  return (s?-1:1)*Infinity;
        return (s?-1:1)*Math.pow(2,e-15)*(1+f/1024);
        }
        let dbgOnce=true;          // print only the first time; flip to false later

    function generateTexture() {
        if (!buffer || !motion || !motionDict) {
            console.log('No buffer or motion data available');
            return;
        }
        const f_buffer = new Float32Array(buffer);
        const u_buffer = new Uint8Array(buffer);

        var texwidth = 1024 * 4; // Set to your desired width
        var texheight = Math.ceil((4 * vertexCount) / (texwidth ));   // 16 words‑per‑splat
        var texdata = new Uint32Array(texwidth * texheight * 4); // 4 components per pixel (RGBA)
        var texdata_c = new Uint8Array(texdata.buffer);
        var texdata_f = new Float32Array(texdata.buffer);


        // Here we convert from a .splat file buffer into a texture
        // With a little bit more foresight perhaps this texture file
        // should have been the native format as it'd be very easy to
        // load it into webgl.
        console.log(texdata.length,        // total words allocated
            (16*vertexCount));     // words you try to write
        // for (let i = 0; i < vertexCount; i++) {
        //      if(dbgOnce && i===0){
        //         const m = motion.subarray(0,8);            // motion words 0‑7
        //         const muSigma = m[7];                      // packed µ | σ
        //         const mu    = halfToFloat( muSigma & 0xffff );
        //         const sigma = halfToFloat( muSigma >>> 16 );
        //         console.log('µ  σ  (splat0) =', mu, sigma);
        //         console.log('motion words 0‑7', [...m]);
        //         dbgOnce = false;
        //     }
        //     // x, y, z
        //     texdata_f[8 * i + 0] = f_buffer[8 * i + 0];
        //     texdata_f[8 * i + 1] = f_buffer[8 * i + 1];
        //     texdata_f[8 * i + 2] = f_buffer[8 * i + 2];

        //     // r, g, b, a
        //     texdata_c[4 * (8 * i + 7) + 0] = u_buffer[32 * i + 24 + 0];
        //     texdata_c[4 * (8 * i + 7) + 1] = u_buffer[32 * i + 24 + 1];
        //     texdata_c[4 * (8 * i + 7) + 2] = u_buffer[32 * i + 24 + 2];
        //     texdata_c[4 * (8 * i + 7) + 3] = u_buffer[32 * i + 24 + 3];

        //     // quaternions
        //     let scale = [
        //         f_buffer[8 * i + 3 + 0],
        //         f_buffer[8 * i + 3 + 1],
        //         f_buffer[8 * i + 3 + 2],
        //     ];
        //     let rot = [
        //         (u_buffer[32 * i + 28 + 0] - 128) / 128,
        //         (u_buffer[32 * i + 28 + 1] - 128) / 128,
        //         (u_buffer[32 * i + 28 + 2] - 128) / 128,
        //         (u_buffer[32 * i + 28 + 3] - 128) / 128,
        //     ];

        //     // Compute the matrix product of S and R (M = S * R)
        //     const M = [
        //         1.0 - 2.0 * (rot[2] * rot[2] + rot[3] * rot[3]),
        //         2.0 * (rot[1] * rot[2] + rot[0] * rot[3]),
        //         2.0 * (rot[1] * rot[3] - rot[0] * rot[2]),

        //         2.0 * (rot[1] * rot[2] - rot[0] * rot[3]),
        //         1.0 - 2.0 * (rot[1] * rot[1] + rot[3] * rot[3]),
        //         2.0 * (rot[2] * rot[3] + rot[0] * rot[1]),

        //         2.0 * (rot[1] * rot[3] + rot[0] * rot[2]),
        //         2.0 * (rot[2] * rot[3] - rot[0] * rot[1]),
        //         1.0 - 2.0 * (rot[1] * rot[1] + rot[2] * rot[2]),
        //     ].map((k, i) => k * scale[Math.floor(i / 3)]);

        //     const sigma = [
        //         M[0] * M[0] + M[3] * M[3] + M[6] * M[6],
        //         M[0] * M[1] + M[3] * M[4] + M[6] * M[7],
        //         M[0] * M[2] + M[3] * M[5] + M[6] * M[8],
        //         M[1] * M[1] + M[4] * M[4] + M[7] * M[7],
        //         M[1] * M[2] + M[4] * M[5] + M[7] * M[8],
        //         M[2] * M[2] + M[5] * M[5] + M[8] * M[8],
        //     ];

        //     texdata[8 * i + 4] = packHalf2x16(4 * sigma[0], 4 * sigma[1]);
        //     texdata[8 * i + 5] = packHalf2x16(4 * sigma[2], 4 * sigma[3]);
        //     texdata[8 * i + 6] = packHalf2x16(4 * sigma[4], 4 * sigma[5]);
        //     texdata.set(motion.subarray(i*8, i*8+8), 16*i + 8);
        //     // texdata.set(texdata.subarray(8*i, 8*i+8), 16*i + 12);

           
        // }
        for (let i = 0; i < vertexCount; ++i) {

            /* ─── log first splat once ─────────────────────────────────── */
            if (dbgOnce && i === 0) {
                const m = motion.subarray(0, 8);           // words 8‑15
                const muSigma = m[7];
                console.log('µ σ =',
                    halfToFloat(muSigma & 0xffff),
                    halfToFloat(muSigma >>> 16));
                console.log('motion words 0‑7', [...m]);
                dbgOnce = false;
            }

            /* ─── words 0‑2 : xyz (bit‑cast float32) ───────────────────── */
            texdata_f[16*i + 0] = f_buffer[8*i + 0];  // x
            texdata_f[16*i + 1] = f_buffer[8*i + 1];  // y
            texdata_f[16*i + 2] = f_buffer[8*i + 2];  // z

            /* ─── read raw numbers from the .splat row ‑‑ no transforms — */
            const sx = f_buffer[8*i + 3 + 0];   // already linear σx
            const sy = f_buffer[8*i + 3 + 1];   // σy
            const sz = f_buffer[8*i + 3 + 2];   // σz

            const qx = (u_buffer[32*i + 28] - 128) / 128.0;
            const qy = (u_buffer[32*i + 29] - 128) / 128.0;
            const qz = (u_buffer[32*i + 30] - 128) / 128.0;
            const qw = (u_buffer[32*i + 31] - 128) / 128.0;
           

            /* ─── word 3 & 4 : quaternion (half16|half16) ─────────────── */
            texdata[16*i + 3] = packHalf2x16(qx, qy);   // rot.x | rot.y
            texdata[16*i + 4] = packHalf2x16(qz, qw);   // rot.z | rot.w

            /* ─── word 5 & 6 : linear σ (half16|half16) ───────────────── */
            texdata[16*i + 5] = packHalf2x16(sx, sy);   // σx | σy
            texdata[16*i + 6] = packHalf2x16(sz, 0.0);  // σz | pad

            /* ─── word 7 : rgba copied byte‑for‑byte ───────────────────── */
            texdata_c[4*(16*i + 7) + 0] = u_buffer[32*i + 24];
            texdata_c[4*(16*i + 7) + 1] = u_buffer[32*i + 25];
            texdata_c[4*(16*i + 7) + 2] = u_buffer[32*i + 26];
            texdata_c[4*(16*i + 7) + 3] = u_buffer[32*i + 27];

            /* ─── words 8‑15 : motion (already Uint32) ‑ copy verbatim — */
            // texdata.set(motion.subarray(i*8, i*8 + 8), 16*i + 8);
            texdata[16*i + 8 + 0] = packHalf2x16(motionDict[i].c1[0],   motionDict[i].c1[1]);
            texdata[16*i + 8 + 1] = packHalf2x16(motionDict[i].c1[2],   motionDict[i].c2[0]);
            texdata[16*i + 8 + 2] = packHalf2x16(motionDict[i].c2[1],   motionDict[i].c2[2]);
            texdata[16*i + 8 + 3] = packHalf2x16(motionDict[i].c3[0],   motionDict[i].c3[1]);
            texdata[16*i + 8 + 4] = packHalf2x16(motionDict[i].c3[2],   0.0);        // pad

            /* ---------- rotation (angular‑vel) ------------------------------ */
            texdata[16*i + 8 + 5] = packHalf2x16(motionDict[i].omega[0], motionDict[i].omega[1]);
            texdata[16*i + 8 + 6] = packHalf2x16(motionDict[i].omega[2], 0.0);       // pad

            /* ---------- temporal RBF (µ, σ) -------------------------------- */
            texdata[16*i + 8 + 7] = packHalf2x16(motionDict[i].mu, motionDict[i].sigma);

            // if (i < 3) {                               // only splats #0–2
            //     const m = motion.subarray(i*8, i*8 + 8);

            //     const h2f = h => halfToFloat(h);       // existing helper

            //     const c1 = [ h2f(m[0] & 0xffff),  h2f(m[0]>>>16), h2f(m[1] & 0xffff) ];
            //     const c2 = [ h2f(m[1]>>>16),      h2f(m[2] & 0xffff), h2f(m[2]>>>16) ];
            //     const c3 = [ h2f(m[3] & 0xffff),  h2f(m[3]>>>16), h2f(m[4] & 0xffff) ];
            //     const  ω = [ h2f(m[5] & 0xffff),  h2f(m[5]>>>16), h2f(m[6] & 0xffff) ];
            //     const μ  = h2f(m[7] & 0xffff);
            //     const σ  = h2f(m[7]>>>16);

            //     console.log(`— splat ${i} —`);
            //     console.log('c1', c1, 'c2', c2, 'c3', c3);
            //     console.log('ω ', ω , 'μσ', μ, σ);
            // }
        }
        
        self.postMessage({ texdata, texwidth, texheight }, [texdata.buffer]);
    }

    function runSort(viewProj) {
        if (!buffer) return;
        const f_buffer = new Float32Array(buffer);
        if (lastVertexCount == vertexCount) {
            let dot =
                lastProj[2] * viewProj[2] +
                lastProj[6] * viewProj[6] +
                lastProj[10] * viewProj[10];
            if (Math.abs(dot - 1) < 0.01) {
                return;
            }
        } else {
            console.log('generating texture')
            generateTexture();
            lastVertexCount = vertexCount;
        }

        console.time("sort");
        let maxDepth = -Infinity;
        let minDepth = Infinity;
        let sizeList = new Int32Array(vertexCount);
        for (let i = 0; i < vertexCount; i++) {
            let depth =
                ((viewProj[2] * f_buffer[8 * i + 0] +
                    viewProj[6] * f_buffer[8 * i + 1] +
                    viewProj[10] * f_buffer[8 * i + 2]) *
                    4096) |
                0;
            sizeList[i] = depth;
            if (depth > maxDepth) maxDepth = depth;
            if (depth < minDepth) minDepth = depth;
        }

        // This is a 16 bit single-pass counting sort
        let depthInv = (256 * 256 - 1) / (maxDepth - minDepth);
        let counts0 = new Uint32Array(256 * 256);
        for (let i = 0; i < vertexCount; i++) {
            sizeList[i] = ((sizeList[i] - minDepth) * depthInv) | 0;
            counts0[sizeList[i]]++;
        }
        let starts0 = new Uint32Array(256 * 256);
        for (let i = 1; i < 256 * 256; i++)
            starts0[i] = starts0[i - 1] + counts0[i - 1];
        depthIndex = new Uint32Array(vertexCount);
        for (let i = 0; i < vertexCount; i++)
            depthIndex[starts0[sizeList[i]]++] = i;

        console.timeEnd("sort");

        lastProj = viewProj;
        self.postMessage({ depthIndex, viewProj, vertexCount }, [
            depthIndex.buffer,
        ]);
    }

    function processPlyBuffer(inputBuffer) {
        const ubuf = new Uint8Array(inputBuffer);
        // 10KB ought to be enough for a header...
        const header = new TextDecoder().decode(ubuf.slice(0, 1024 * 10));
        const header_end = "end_header\n";
        const header_end_index = header.indexOf(header_end);
        if (header_end_index < 0)
            throw new Error("Unable to read .ply file header");
        const vertexCount = parseInt(/element vertex (\d+)\n/.exec(header)[1]);
        console.log("Vertex Count", vertexCount);
        let row_offset = 0,
            offsets = {},
            types = {};
        const TYPE_MAP = {
            double: "getFloat64",
            int: "getInt32",
            uint: "getUint32",
            float: "getFloat32",
            short: "getInt16",
            ushort: "getUint16",
            uchar: "getUint8",
        };
        for (let prop of header
            .slice(0, header_end_index)
            .split("\n")
            .filter((k) => k.startsWith("property "))) {
            const [p, type, name] = prop.split(" ");
            const arrayType = TYPE_MAP[type] || "getInt8";
            types[name] = arrayType;
            offsets[name] = row_offset;
            row_offset += parseInt(arrayType.replace(/[^\d]/g, "")) / 8;
        }
        console.log("Bytes per row", row_offset, types, offsets);

        let dataView = new DataView(
            inputBuffer,
            header_end_index + header_end.length,
        );
        let row = 0;
        const attrs = new Proxy(
            {},
            {
                get(target, prop) {
                    if (!types[prop]) throw new Error(prop + " not found");
                    return dataView[types[prop]](
                        row * row_offset + offsets[prop],
                        true,
                    );
                },
            },
        );

        console.time("calculate importance");
        let sizeList = new Float32Array(vertexCount);
        let sizeIndex = new Uint32Array(vertexCount);
        for (row = 0; row < vertexCount; row++) {
            sizeIndex[row] = row;
            if (!types["scale_0"]) continue;
            const size =
                Math.exp(attrs.scale_0) *
                Math.exp(attrs.scale_1) *
                Math.exp(attrs.scale_2);
            const opacity = 1 / (1 + Math.exp(-attrs.opacity));
            sizeList[row] = size * opacity;
        }
        console.timeEnd("calculate importance");

        console.time("sort");
        sizeIndex.sort((b, a) => sizeList[a] - sizeList[b]);
        console.timeEnd("sort");

        // 6*4 + 4 + 4 = 8*4
        // XYZ - Position (Float32)
        // XYZ - Scale (Float32)
        // RGBA - colors (uint8)
        // IJKL - quaternion/rot (uint8)
        const rowLength = 3 * 4 + 3 * 4 + 4 + 4;
        const buffer = new ArrayBuffer(rowLength * vertexCount);

        console.time("build buffer");
        for (let j = 0; j < vertexCount; j++) {
            row = sizeIndex[j];

            const position = new Float32Array(buffer, j * rowLength, 3);
            const scales = new Float32Array(buffer, j * rowLength + 4 * 3, 3);
            const rgba = new Uint8ClampedArray(
                buffer,
                j * rowLength + 4 * 3 + 4 * 3,
                4,
            );
            const rot = new Uint8ClampedArray(
                buffer,
                j * rowLength + 4 * 3 + 4 * 3 + 4,
                4,
            );

            if (types["scale_0"]) {
                const qlen = Math.sqrt(
                    attrs.rot_0 ** 2 +
                        attrs.rot_1 ** 2 +
                        attrs.rot_2 ** 2 +
                        attrs.rot_3 ** 2,
                );

                rot[0] = (attrs.rot_0 / qlen) * 128 + 128;
                rot[1] = (attrs.rot_1 / qlen) * 128 + 128;
                rot[2] = (attrs.rot_2 / qlen) * 128 + 128;
                rot[3] = (attrs.rot_3 / qlen) * 128 + 128;

                scales[0] = Math.exp(attrs.scale_0);
                scales[1] = Math.exp(attrs.scale_1);
                scales[2] = Math.exp(attrs.scale_2);
            } else {
                scales[0] = 0.01;
                scales[1] = 0.01;
                scales[2] = 0.01;

                rot[0] = 255;
                rot[1] = 0;
                rot[2] = 0;
                rot[3] = 0;
            }

            position[0] = attrs.x;
            position[1] = attrs.y;
            position[2] = attrs.z;

            if (types["f_dc_0"]) {
                const SH_C0 = 0.28209479177387814;
                rgba[0] = (0.5 + SH_C0 * attrs.f_dc_0) * 255;
                rgba[1] = (0.5 + SH_C0 * attrs.f_dc_1) * 255;
                rgba[2] = (0.5 + SH_C0 * attrs.f_dc_2) * 255;
            } else {
                rgba[0] = attrs.red;
                rgba[1] = attrs.green;
                rgba[2] = attrs.blue;
            }
            if (types["opacity"]) {
                rgba[3] = (1 / (1 + Math.exp(-attrs.opacity))) * 255;
            } else {
                rgba[3] = 255;
            }
        }
        console.timeEnd("build buffer");
        return buffer;
    }

    const throttledSort = () => {
        if (!sortRunning) {
            sortRunning = true;
            let lastView = viewProj;
            runSort(lastView);
            setTimeout(() => {
                sortRunning = false;
                if (lastView !== viewProj) {
                    throttledSort();
                }
            }, 0);
        }
    };

    let sortRunning;
    self.onmessage = (e) => {
        if (e.data.ply) {
            vertexCount = 0;
            runSort(viewProj);
            buffer = processPlyBuffer(e.data.ply);
            vertexCount = Math.floor(buffer.byteLength / rowLength);
            postMessage({ buffer: buffer, save: !!e.data.save });
        } else if (e.data.buffer) {
            buffer = e.data.buffer;
            vertexCount = e.data.vertexCount;
        } else if (e.data.vertexCount) {
            vertexCount = e.data.vertexCount;
        } else if (e.data.view) {
            viewProj = e.data.view;
            throttledSort();
        } else if (e.data.motion) {              // ← arrives once, right after fetch
            motion = new Uint32Array(e.data.motion);   // keep a typed view
            // --- build a JS dictionary { index -> decodedObject } -------------------
            motionDict = {};
            for (let i = 0; i < motion.length / 8; ++i) {
                motionDict[i] = decodeMotionRow(motion, i);
            }
            console.log("motionDict row 1: ", motionDict[1]);
            generateTexture();
        }
    };
}

const vertexShaderSource = `
  #version 300 es
  precision highp float;
  precision highp int;
  
  uniform highp usampler2D u_texture;
  uniform mat4 projection, view;
  uniform vec2 focal;
  uniform vec2 viewport;
  uniform float time;
  
  in vec2 position;
  in int index;
  
  out vec4 vColor;
  out vec2 vPosition;
  
  void main () {
      gl_Position = vec4(0.0, 0.0, 2.0, 1.0);

      uvec4 motion1 = texelFetch(u_texture, ivec2(((uint(index) & 0x3ffu) << 2) | 3u, uint(index) >> 10), 0);
      vec2 trbf = unpackHalf2x16(motion1.w);
      float dt = time - trbf.x;

      float topacity = exp(-1.0 * pow(dt / trbf.y, 2.0));
      if(topacity < 0.02) return;

      uvec4 motion0 = texelFetch(u_texture, ivec2(((uint(index) & 0x3ffu) << 2) | 2u, uint(index) >> 10), 0);
      uvec4 static0 = texelFetch(u_texture, ivec2(((uint(index) & 0x3ffu) << 2), uint(index) >> 10), 0);

      vec2 m0 = unpackHalf2x16(motion0.x), m1 = unpackHalf2x16(motion0.y), m2 = unpackHalf2x16(motion0.z), 
           m3 = unpackHalf2x16(motion0.w), m4 = unpackHalf2x16(motion1.x); 
      
      vec4 trot = vec4(unpackHalf2x16(motion1.y).xy, unpackHalf2x16(motion1.z).xy) * dt;
      vec3 tpos = (vec3(m0.xy, m1.x) * dt + vec3(m1.y, m2.xy) * dt*dt + vec3(m3.xy, m4.x) * dt*dt*dt);
      
      vec4 cam = view * vec4(uintBitsToFloat(static0.xyz) + tpos, 1);
      vec4 pos = projection * cam;
  
      float clip = 1.2 * pos.w;
      if (pos.z < -clip || pos.x < -clip || pos.x > clip || pos.y < -clip || pos.y > clip) return;
      uvec4 static1 = texelFetch(u_texture, ivec2(((uint(index) & 0x3ffu) << 2) | 1u, uint(index) >> 10), 0);

      vec4 rot = vec4(unpackHalf2x16(static0.w).xy, unpackHalf2x16(static1.x).xy) + trot;
      vec3 scale = vec3(unpackHalf2x16(static1.y).xy, unpackHalf2x16(static1.z).x);
      rot /= sqrt(dot(rot, rot));
  
      mat3 S = mat3(scale.x, 0.0, 0.0, 0.0, scale.y, 0.0, 0.0, 0.0, scale.z);
      mat3 R = mat3(
        1.0 - 2.0 * (rot.z * rot.z + rot.w * rot.w), 2.0 * (rot.y * rot.z - rot.x * rot.w), 2.0 * (rot.y * rot.w + rot.x * rot.z),
        2.0 * (rot.y * rot.z + rot.x * rot.w), 1.0 - 2.0 * (rot.y * rot.y + rot.w * rot.w), 2.0 * (rot.z * rot.w - rot.x * rot.y),
        2.0 * (rot.y * rot.w - rot.x * rot.z), 2.0 * (rot.z * rot.w + rot.x * rot.y), 1.0 - 2.0 * (rot.y * rot.y + rot.z * rot.z));
      mat3 M = S * R;
      mat3 Vrk = 4.0 * transpose(M) * M;
      mat3 J = mat3(
          focal.x / cam.z, 0., -(focal.x * cam.x) / (cam.z * cam.z), 
          0., -focal.y / cam.z, (focal.y * cam.y) / (cam.z * cam.z), 
          0., 0., 0.
      );
  
      mat3 T = transpose(mat3(view)) * J;
      mat3 cov2d = transpose(T) * Vrk * T;
  
      float mid = (cov2d[0][0] + cov2d[1][1]) / 2.0;
      float radius = length(vec2((cov2d[0][0] - cov2d[1][1]) / 2.0, cov2d[0][1]));
      float lambda1 = mid + radius, lambda2 = mid - radius;
  
      if(lambda2 < 0.0) return;
      vec2 diagonalVector = normalize(vec2(cov2d[0][1], lambda1 - cov2d[0][0]));
      vec2 majorAxis = min(sqrt(2.0 * lambda1), 1024.0) * diagonalVector;
      vec2 minorAxis = min(sqrt(2.0 * lambda2), 1024.0) * vec2(diagonalVector.y, -diagonalVector.x);
      
      uint rgba = static1.w;
      vColor = 
        clamp(pos.z/pos.w+1.0, 0.0, 1.0) * 
        vec4(1.0, 1.0, 1.0, topacity) *
        vec4(
          (rgba) & 0xffu, 
          (rgba >> 8) & 0xffu, 
          (rgba >> 16) & 0xffu, 
          (rgba >> 24) & 0xffu) / 255.0;

      vec2 vCenter = vec2(pos) / pos.w;
      gl_Position = vec4(
          vCenter 
          + position.x * majorAxis / viewport 
          + position.y * minorAxis / viewport, 0.0, 1.0);

      vPosition = position;
  }
  `.trim();



const fragmentShaderSource = `
  #version 300 es
  precision highp float;
  
  in vec4 vColor;
  in vec2 vPosition;
  
  out vec4 fragColor;
  
  void main () {
      float A = -dot(vPosition, vPosition);
      if (A < -4.0) discard;
      float B = exp(A) * vColor.a;
      fragColor = vec4(B * vColor.rgb, B);
  }
  
  `.trim();

let defaultViewMatrix = [
    0.47, 0.04, 0.88, 0, -0.11, 0.99, 0.02, 0, -0.88, -0.11, 0.47, 0, 0.07,
    0.03, 6.55, 1,
];
let viewMatrix = defaultViewMatrix;

const START_POS = [0.5828, -0.0901, 1.6495];
const END_POS   = [1.3242, -0.3213, 4.0102];



let   lerpT     = 0;                               // 0 = START, 1 = END
let   camPos    = [...START_POS];
let yaw   = 0.35;
let pitch = 0.0006108470632345802;

ROTATE_SPEED = 1.7

let targetT      = lerpT;             // wheel sets this
let targetYaw    = yaw;               // drag sets this
let targetPitch  = pitch;

let currentT     = lerpT;
let currentYaw   = yaw;
let currentPitch = pitch;

let dragging = false;
let px = 0, py = 0;            // last pointer position (normalised –0.5‥0.5)
let pinch0 = 0, startT = 0;    // two-finger pinch start

updateFocalLengths();

function posAlongPath(points, t) {
  const n = points.length - 1;          // segments = way-points – 1
  const scaled = t * n;                 // 0---n
  const i = Math.min(Math.floor(scaled), n - 1);   // current segment index
  const localT = scaled - i;            // 0---1 inside that segment

  const a = points[i];
  const b = points[i + 1];

  // simple linear interpolation; swap in Catmull-Rom, Bézier, etc. if you
  // want a smooth curve instead of straight lines
  return [
    a[0] + (b[0] - a[0]) * localT,
    a[1] + (b[1] - a[1]) * localT,
    a[2] + (b[2] - a[2]) * localT,
  ];
}

function distanceBetweenPointers(ev) {
  // ev.getCoalescedEvents() not needed – we can query the active touches:
  if (ev.pointerType !== 'touch') return 0;
  const touches = [...ev.targetTouches || []];
  if (touches.length < 2) return 0;
  const dx = touches[0].clientX - touches[1].clientX;
  const dy = touches[0].clientY - touches[1].clientY;
  return Math.hypot(dx, dy);
}

function samplePath(path, t) {
  const n = path.length - 1;
  const s = Math.min(t * n, n - 1e-6);
  const i = Math.floor(s);
  const u = s - i;

  const a = path[i], b = path[i + 1];

  // linear pos
  const pos = [
    a.pos[0] + (b.pos[0] - a.pos[0]) * u,
    a.pos[1] + (b.pos[1] - a.pos[1]) * u,
    a.pos[2] + (b.pos[2] - a.pos[2]) * u,
  ];

  // shortest-arc yaw blend (wrap around –π…π)
  const dyaw = wrapPi(b.yaw - a.yaw);
  const yaw  = wrapPi(a.yaw + dyaw * u);

  // pitch is small, plain lerp is fine
  const pitch = a.pitch + (b.pitch - a.pitch) * u;

  return { pos, yaw, pitch };
}

const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));

const LERP_RATE = 8;

function lerp(a, b, t) { return a + (b - a) * t; }

function updateCamPos() {
  camPos = [
    lerp(START_POS[0], END_POS[0], lerpT),
    lerp(START_POS[1], END_POS[1], lerpT),
    lerp(START_POS[2], END_POS[2], lerpT),
  ];
  console.log("updateCamPos", camPos);
  console.log("yaw", yaw, "pitch", pitch);
  viewMatrix = makeViewMatrix(yaw, pitch, camPos);
}
updateCamPos(); 

let camCur, camDst, tBlend = 1;

const YAW_RANGE   = Math.PI/2;        // ±180°   (tweak to taste)
const PITCH_RANGE = Math.PI / 2;

let yawFrac   = 0.5;   // 0 = far left, 1 = far right, 0.5 = centre
let pitchFrac = 0.5;   // 0 = bottom,    1 = top


const slider = document.getElementById('moveSlider');

function updateTick () {
    const t    = slider.value / 100;               // 0‥1
    targetT = t;
}

slider.addEventListener('input',  updateTick);
window.addEventListener('resize', updateTick);
updateTick();
let worker;

const f16 = (() => {
  const f32 = new Float32Array(1);
  const u32 = new Uint32Array(f32.buffer);
  return h => {
    const s =  (h & 0x8000) << 16;
    let   e = (h & 0x7C00) >> 10;
    let   f =  h & 0x03FF;
    if (!e) {                        // sub‑normal / zero
      if (!f) { u32[0] = s; return f32[0]; }
      e = 1; while (!(f & 0x400)) { f <<= 1; --e; }
      f &= 0x3FF;
    } else if (e === 31) {           // Inf / NaN
      u32[0] = s | 0x7F800000 | (f << 13); return f32[0];
    }
    e = e + (127 - 15);
    u32[0] = s | (e << 23) | (f << 13);
    return f32[0];
  };
})();

// ---------------------------------------------------------------------------
//  Decode one splat (8 × uint32) -> plain object
function decodeMotionRow(u32, idx) {
  const base = idx * 8;
  const lo = word => word & 0xFFFF;
  const hi = word => word >>> 16;

  const w = n => u32[base + n];        // convenience alias

  return {
    c1:    [ f16(lo(w(0))), f16(hi(w(0))), f16(lo(w(1))) ],
    c2:    [ f16(hi(w(1))), f16(lo(w(2))), f16(hi(w(2))) ],
    c3:    [ f16(lo(w(3))), f16(hi(w(3))), f16(lo(w(4))) ],
    omega: [ f16(lo(w(5))), f16(hi(w(5))), f16(lo(w(6))) ],
    mu:     f16(lo(w(7))),
    sigma:  f16(hi(w(7))),
  };
}

// ---------------------------------------------------------------------------
//  New loadMotion — creates `motionDict` for debugging
async function loadMotion(url) {
  const resp = await fetch(url);                 // e.g. "alleyway.motion.bin"
  if (!resp.ok) {
    console.error('motion file not found:', resp.status, resp.url);
    return;
  }

  const buf = await resp.arrayBuffer();          // raw binary
  const u32  = new Uint32Array(buf);             // 8 × uint32 per splat
  const splatCount = u32.length / 8;

  // --- build a JS dictionary { index -> decodedObject } -------------------
  const motionDict = {};
  for (let i = 0; i < splatCount; ++i) {
    motionDict[i] = decodeMotionRow(u32, i);
  }

  // quick peek
  console.log('decoded motion rows:', splatCount);
  console.log('row 0', motionDict[0]);
  console.log('row 1', motionDict[1]);
  console.log('row 2', motionDict[2]);

  // you can keep a reference globally if you wish:
  window.motionDebug = motionDict;   // <— inspect from DevTools console

  // still pass the original buffer to the worker for real‑time rendering
  worker.postMessage({ motion: buf }, [buf]);    // transferable
}

async function main() {
    console.log("main() entered");

    let carousel = true;
    try {
        viewMatrix = JSON.parse(decodeURIComponent(location.hash.slice(1)));
        carousel = false;
    } catch (err) {}
    
    const req = await fetch(url, {
        mode: "cors", // no-cors, *cors, same-origin
        credentials: "omit", // include, *same-origin, omit
    });
    console.log(req);
    if (req.status != 200)
        throw new Error(req.status + " Unable to load " + req.url);


    const headerBuf  = await req.clone().arrayBuffer();        // full file copy

   
    const rowLength = 3 * 4 + 3 * 4 + 4 + 4;
    const reader = req.body.getReader();
    // let splatData = new Uint8Array(req.headers.get("content-length"));
    let splatData = new Uint8Array(1024 * 1024);   // 1 MiB

    var downsample =1;


    worker = new Worker(
        URL.createObjectURL(
            new Blob(["(", createWorker.toString(), ")(self)"], {
                type: "application/javascript",
            }),
        ),
    );

    loadMotion(url.split('.').slice(0,-1).join('.') + '.motion.bin').catch(console.error);

    const canvas = document.getElementById("canvas");
    const fps = document.getElementById("fps");
    const camid = document.getElementById("camid");

    let projectionMatrix;

    const gl = canvas.getContext("webgl2", {
        antialias: false,
    });

    const vertexShader = gl.createShader(gl.VERTEX_SHADER);
    gl.shaderSource(vertexShader, vertexShaderSource);
    gl.compileShader(vertexShader);
    if (!gl.getShaderParameter(vertexShader, gl.COMPILE_STATUS))
        console.error(gl.getShaderInfoLog(vertexShader));

    const fragmentShader = gl.createShader(gl.FRAGMENT_SHADER);
    gl.shaderSource(fragmentShader, fragmentShaderSource);
    gl.compileShader(fragmentShader);
    if (!gl.getShaderParameter(fragmentShader, gl.COMPILE_STATUS))
        console.error(gl.getShaderInfoLog(fragmentShader));

    const program = gl.createProgram();
    gl.attachShader(program, vertexShader);
    gl.attachShader(program, fragmentShader);
    gl.linkProgram(program);
    gl.useProgram(program);

    if (!gl.getProgramParameter(program, gl.LINK_STATUS))
        console.error(gl.getProgramInfoLog(program));

    gl.disable(gl.DEPTH_TEST); // Disable depth testing

    // Enable blending
    gl.enable(gl.BLEND);
    gl.blendFuncSeparate(
        gl.ONE_MINUS_DST_ALPHA,
        gl.ONE,
        gl.ONE_MINUS_DST_ALPHA,
        gl.ONE,
    );
    gl.blendEquationSeparate(gl.FUNC_ADD, gl.FUNC_ADD);

    const u_projection = gl.getUniformLocation(program, "projection");
    const u_viewport = gl.getUniformLocation(program, "viewport");
    const u_focal = gl.getUniformLocation(program, "focal");
    const u_view = gl.getUniformLocation(program, "view");
    const u_time       = gl.getUniformLocation(program, "time");

    // positions
    const triangleVertices = new Float32Array([-2, -2, 2, -2, 2, 2, -2, 2]);
    const vertexBuffer = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, vertexBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, triangleVertices, gl.STATIC_DRAW);
    const a_position = gl.getAttribLocation(program, "position");
    gl.enableVertexAttribArray(a_position);
    gl.bindBuffer(gl.ARRAY_BUFFER, vertexBuffer);
    gl.vertexAttribPointer(a_position, 2, gl.FLOAT, false, 0, 0);

    var texture = gl.createTexture();
    gl.bindTexture(gl.TEXTURE_2D, texture);

    var u_textureLocation = gl.getUniformLocation(program, "u_texture");
    gl.uniform1i(u_textureLocation, 0);

    const indexBuffer = gl.createBuffer();
    const a_index = gl.getAttribLocation(program, "index");
    gl.enableVertexAttribArray(a_index);
    gl.bindBuffer(gl.ARRAY_BUFFER, indexBuffer);
    gl.vertexAttribIPointer(a_index, 1, gl.INT, false, 0, 0);
    gl.vertexAttribDivisor(a_index, 1);

    const resize = () => {
        updateFocalLengths();
        gl.uniform2fv(u_focal, new Float32Array([fx, fy]));

        /* --- projection --- */
        projectionMatrix = getProjectionMatrix(
            fx, fy,
            innerWidth, innerHeight
        );
        // gl.uniform2fv(u_focal, new Float32Array([camera.fx, camera.fy]));

        // projectionMatrix = getProjectionMatrix(
        //     camera.fx,
        //     camera.fy,
        //     innerWidth,
        //     innerHeight,
        // );

        gl.uniform2fv(u_viewport, new Float32Array([innerWidth, innerHeight]));

        gl.canvas.width = Math.round(innerWidth / downsample);
        gl.canvas.height = Math.round(innerHeight / downsample);
        gl.viewport(0, 0, gl.canvas.width, gl.canvas.height);

        gl.uniformMatrix4fv(u_projection, false, projectionMatrix);
    };

    window.addEventListener("resize", resize);
    resize();

    worker.onmessage = (e) => {
        if (e.data.buffer) {
            splatData = new Uint8Array(e.data.buffer);
            if (e.data.save) {
                const blob = new Blob([splatData.buffer], {
                    type: "application/octet-stream",
                });
                const link = document.createElement("a");
                link.download = "model.splat";
                link.href = URL.createObjectURL(blob);
                document.body.appendChild(link);
                link.click();
            }
        } else if (e.data.texdata) {
            const { texdata, texwidth, texheight } = e.data;
            // console.log(texdata)
            gl.bindTexture(gl.TEXTURE_2D, texture);
            gl.texParameteri(
                gl.TEXTURE_2D,
                gl.TEXTURE_WRAP_S,
                gl.CLAMP_TO_EDGE,
            );
            gl.texParameteri(
                gl.TEXTURE_2D,
                gl.TEXTURE_WRAP_T,
                gl.CLAMP_TO_EDGE,
            );
            gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.NEAREST);
            gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.NEAREST);

            gl.texImage2D(
                gl.TEXTURE_2D,
                0,
                gl.RGBA32UI,
                texwidth,
                texheight,
                0,
                gl.RGBA_INTEGER,
                gl.UNSIGNED_INT,
                texdata,
            );
            gl.activeTexture(gl.TEXTURE0);
            gl.bindTexture(gl.TEXTURE_2D, texture);
        } else if (e.data.depthIndex) {
            const { depthIndex, viewProj } = e.data;
            gl.bindBuffer(gl.ARRAY_BUFFER, indexBuffer);
            gl.bufferData(gl.ARRAY_BUFFER, depthIndex, gl.DYNAMIC_DRAW);
            vertexCount = e.data.vertexCount;
        }
    };


    function getCameraDistanceFromOrigin(viewMatrix) {
        // 1. Invert the view matrix to get the camera transform
        const inv = invert4(viewMatrix);
      
        // 2. The camera's world position is (inv[12], inv[13], inv[14]) in column-major order.
        const x = inv[12];
        const y = inv[13];
        const z = inv[14];
      
        // 3. Distance from the origin:
        return Math.sqrt(x*x + y*y + z*z);
      }

 

    canvas.style.touchAction = 'none';   // browser must NOT translate swipes into scroll/zoom

    window.addEventListener("wheel", e => {
        const dir  = Math.sign(e.deltaY);     // −1 up, +1 down
        const step = 0.02 / PATH.length;                    // feel free to tweak
        targetT = Math.max(0, Math.min(1, targetT + dir * step));
        //update slider
        slider.value = targetT * 100;         // 0‥100
        }, { passive:true });

    let startX, startY, down;
    // canvas.addEventListener("mousedown", (e) => {
    //     carousel = false;
    //     e.preventDefault();
    //     // updateYawPitchFromView();              // <── NEW: sync angles right now
    //     syncFromView();
    //     startX = e.clientX;
    //     startY = e.clientY;
    //     down = e.ctrlKey || e.metaKey ? 2 : 1;
    
    // });
    canvas.addEventListener("contextmenu", (e) => {
        e.preventDefault(); // No browser menu on right-click
    });
    
    // On mouse down, decide rotate vs pan
    // canvas.addEventListener("mousedown", (e) => {
    //     e.preventDefault();
    //     carousel = false;
    
    //     startX = e.clientX;
    //     startY = e.clientY;
    
    //     if (e.button === 2) {
    //         // Right click => pan
    //         down = 2;
    //     } else if (e.button === 0) {
    //         // Left click => rotate
    //         down = 1;
    //     }
    // });

    // canvas.addEventListener("mousemove", (e) => {
    //     // e.preventDefault();
    //     const x =   (e.clientX) / innerWidth - 0.5;   // horiz drag
    //     const y =   -(e.clientY) / innerHeight + 0.5;  // vert  drag

    //     targetYaw  = (MAX_YAW - MIN_YAW) * x;                    // apply delta …

    //     targetPitch = (MAX_PITCH - MIN_PITCH) * y;               // apply delta

    //     console.log("targetYaw", targetYaw, "targetPitch", targetPitch);
    //     console.log("e.clientX", e.clientX, "e.clientY", e.clientY);

    //     // startX = e.clientX;     // reset drag origin for next event
    //     // startY = e.clientY;
    // });
    // canvas.addEventListener("mouseup", (e) => {
    //     e.preventDefault();
    //     down = false;
    //     startX = 0;
    //     startY = 0;
    //     if ((down === 2 && e.button === 2) || (down === 1 && e.button === 0)) {
    //         down = false;
    //     }
    // });

    canvas.addEventListener('pointerdown', ev => {
    canvas.setPointerCapture(ev.pointerId);

    if (ev.isPrimary) {               // first finger / mouse button
        dragging = true;
        px = ev.clientX;
        py = ev.clientY;
    } else {                          // second finger → pinch
        pinch0  = getPinchDistance(ev);
        startT  = targetT;
    }
    });

    canvas.addEventListener('pointermove', ev => {
    if (!dragging) return;

    /* ----- rotation (1st pointer) ----------------------------------- */
    if ((ev.buttons & 1) || (ev.pointerType === 'touch' && ev.isPrimary)) {
        const dx = (ev.clientX - px) / innerWidth;
        const dy = (ev.clientY - py) / innerHeight;

        yawFrac   = clamp(yawFrac   + dx * ROTATE_SPEED, 0, 1);
        pitchFrac = clamp(pitchFrac - dy * ROTATE_SPEED, 0, 1);

        px = ev.clientX;
        py = ev.clientY;
  }
    
    });

    canvas.addEventListener('pointerup',     ev => { if (ev.isPrimary) dragging = false; });
    canvas.addEventListener('pointercancel', ev => { if (ev.isPrimary) dragging = false; });


    
    let jumpDelta = 0;
    let vertexCount = 0;

    let lastFrame = 0;
    let avgFps = 0;
    let start = 0;

    const PERIOD_S = 1.0;                // seconds for one complete loop
    const startMs  = performance.now();  // remember program start
    
    const frame = (now) => {
        // console.log("frame", now);

    //    updateCamPos();
         const tSec  = 0.0001 * (now - startMs);        // wall clock → s
        const phase = (tSec % PERIOD_S) / PERIOD_S;   // 0‑‑1 repeating
        gl.uniform1f(u_time, phase);
        const dt = (now - lastFrame) * 0.001;            // ms → s
        const k  = 1.0 - Math.exp(-LERP_RATE * dt);      // time-independent factor

        currentT     += (targetT     - currentT)     * k;
        currentYaw   += wrapPi(targetYaw - currentYaw)   * k;
        currentPitch += (targetPitch - currentPitch)     * k;


        // camPos = posAlongPath(PATH, currentT);

        // viewMatrix = makeViewMatrix(currentYaw, currentPitch, camPos);

        const base = samplePath(PATH, currentT);

        const yaw   = base.yaw   + (yawFrac   - 0.5) * YAW_RANGE;
        const pitch = base.pitch + (pitchFrac - 0.5) * PITCH_RANGE;

        viewMatrix  = makeViewMatrix(yaw, pitch, base.pos);



        // syncFromView();

        // let inv = invert4(viewMatrix);

        // viewMatrix = invert4(inv);

        // if (carousel) {
        //     let inv = invert4(defaultViewMatrix);

        //     const t = Math.sin((Date.now() - start) / 5000);
        //     inv = translate4(inv, 2.5 * t, 0, 6 * (1 - Math.cos(t)));
        //     inv = rotate4(inv, -0.6 * t, 0, 1, 0);

        //     viewMatrix = invert4(inv);
        // }

        let inv2 = invert4(viewMatrix);
        inv2 = translate4(inv2, 0, -jumpDelta, 0);
        inv2 = rotate4(inv2, -0.1 * jumpDelta, 1, 0, 0);
        let actualViewMatrix = invert4(inv2);

        const viewProj = multiply4(projectionMatrix, actualViewMatrix);
        worker.postMessage({ view: viewProj });

        const currentFps = 1000 / (now - lastFrame) || 0;
        avgFps = avgFps * 0.9 + currentFps * 0.1;

        if (vertexCount > 0) {
            document.getElementById("spinner").style.display = "none";
            gl.uniformMatrix4fv(u_view, false, actualViewMatrix);
            gl.clear(gl.COLOR_BUFFER_BIT);
            gl.drawArraysInstanced(gl.TRIANGLE_FAN, 0, 4, vertexCount);
        } else {
            gl.clear(gl.COLOR_BUFFER_BIT);
            document.getElementById("spinner").style.display = "";
            start = Date.now() + 2000;
        }
        const progress = (100 * vertexCount) / (splatData.length / rowLength);
        if (progress < 100) {
            document.getElementById("progress").style.width = progress + "%";
        } else {
            document.getElementById("progress").style.display = "none";
        }
        fps.innerText = Math.round(avgFps) + " fps";
        
        lastFrame = now;

        
        requestAnimationFrame(frame);
    };

    requestAnimationFrame(frame);

    const isPly = (splatData) =>
        splatData[0] == 112 &&
        splatData[1] == 108 &&
        splatData[2] == 121 &&
        splatData[3] == 10;

    const selectFile = (file) => {
        const fr = new FileReader();
        if (/\.json$/i.test(file.name)) {
            fr.onload = () => {
                cameras = JSON.parse(fr.result);
                viewMatrix = getViewMatrix(cameras[0]);
                projectionMatrix = getProjectionMatrix(
                    camera.fx / downsample,
                    camera.fy / downsample,
                    canvas.width,
                    canvas.height,
                );
                gl.uniformMatrix4fv(u_projection, false, projectionMatrix);

                console.log("Loaded Cameras");
            };
            fr.readAsText(file);
        } else {
            stopLoading = true;
            fr.onload = () => {
                splatData = new Uint8Array(fr.result);
                console.log("Loaded", Math.floor(splatData.length / rowLength));

                if (isPly(splatData)) {
                    // ply file magic header means it should be handled differently
                    worker.postMessage({ ply: splatData.buffer, save: true });
                } else {
                    worker.postMessage({
                        buffer: splatData.buffer,
                        vertexCount: Math.floor(splatData.length / rowLength),
                    });
                }
            };
            fr.readAsArrayBuffer(file);
        }
    };

    window.addEventListener("hashchange", (e) => {
        try {
            viewMatrix = JSON.parse(decodeURIComponent(location.hash.slice(1)));
            carousel = false;
        } catch (err) {}
    });

    const preventDefault = (e) => {
        e.preventDefault();
        e.stopPropagation();
    };
    document.addEventListener("dragenter", preventDefault);
    document.addEventListener("dragover", preventDefault);
    document.addEventListener("dragleave", preventDefault);
    document.addEventListener("drop", (e) => {
        e.preventDefault();
        e.stopPropagation();
        selectFile(e.dataTransfer.files[0]);
    });

    let bytesRead = 0;
    let lastVertexCount = -1;
    let stopLoading = false;

    while (true) {
        const { done, value } = await reader.read();
        if (done || stopLoading) break;

        if (bytesRead + value.length > splatData.length) {
            const nextSize = Math.max(
                splatData.length * 2,
                bytesRead + value.length
            );
            const bigger = new Uint8Array(nextSize);
            bigger.set(splatData);          // copy old data
            splatData = bigger;
        }

        splatData.set(value, bytesRead);
        bytesRead += value.length;

        if (vertexCount > lastVertexCount) {
            if (!isPly(splatData)) {
                worker.postMessage({
                    buffer: splatData.buffer,
                    vertexCount: Math.floor(bytesRead / rowLength),
                });
            }
            lastVertexCount = vertexCount;
        }
    }
    if (!stopLoading) {
        if (isPly(splatData)) {
            // ply file magic header means it should be handled differently
            worker.postMessage({ ply: splatData.buffer, save: false });
        } else {
            worker.postMessage({
                buffer: splatData.buffer,
                vertexCount: Math.floor(bytesRead / rowLength),
            });
        }
    }
    downsample =
        splatData.length / rowLength > 500000 ? 1 : 1 / devicePixelRatio;
    console.log(splatData.length / rowLength, downsample);
}

let cameras;
let z = 0;
const params = new URLSearchParams(location.search);
var url;
if (params.get("obj")){
    url = params.get("obj");
}
else{
    url = 'point_cloud.splat';
}

fetch(url.split(".")[0] + "_cameras.json")
  .then(r => r.json())
  .then(json => {

    cameras = json;
    


   cameras.forEach(cam => {
    const R = [
        [cam.rotation[0][0], cam.rotation[0][1], cam.rotation[0][2]],
        [cam.rotation[1][0], cam.rotation[1][1], cam.rotation[1][2]],
        [cam.rotation[2][0], cam.rotation[2][1], cam.rotation[2][2]],
    ];
    const [yaw, pitch] = yawPitchFromRot3(R);   // helper you already have

    // invert Y & Z to match your world handedness
    PATH.push({
        pos: [ cam.position[0], -cam.position[1], -cam.position[2] ],
        yaw,
        pitch,
    });
    });
    

    console.log("length of PATH", PATH.length); 

     main().catch((err) => {
        document.getElementById("spinner").style.display = "none";
        document.getElementById("message").innerText = err.toString();
        });
   
  });
