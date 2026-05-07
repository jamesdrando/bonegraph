'use strict';
// =============================================================================
// Section 1: CONSTANTS
// =============================================================================
const EDGE = {
  DIRECTED: 'directed',
  BIDIRECTIONAL: 'bidirectional',
  ELASTIC: 'elastic'
};
const TOOL = {
  SELECT: 'select',
  ADD_NODE: 'add_node',
  ADD_EDGE: 'add_edge',
  DELETE: 'delete'
};
const EXPORT_PRESET = {
  SIDE_2D: 'side_2d',
  ISO_4: 'iso_4',
  ISO_6: 'iso_6',
  ISO_8: 'iso_8',
  CUSTOM: 'custom'
};
const NODE_RADIUS          = 8;
const EXPORT_NODE_DIAMETER = 4;
const HIT_RADIUS           = 12;
const EDGE_HIT_WIDTH       = 6;
const PARALLEL_RAY_DISTANCE = 1000; // fallback extent when ray is parallel to target plane
// =============================================================================
// Section 2: VEC3 MATH
// =============================================================================
const Vec3 = {
  add(a, b) { return { x: a.x + b.x, y: a.y + b.y, z: a.z + b.z }; },
  sub(a, b) { return { x: a.x - b.x, y: a.y - b.y, z: a.z - b.z }; },
  scale(v, s) { return { x: v.x * s, y: v.y * s, z: v.z * s }; },
  dot(a, b) { return a.x * b.x + a.y * b.y + a.z * b.z; },
  cross(a, b) {
    return {
      x: a.y * b.z - a.z * b.y,
      y: a.z * b.x - a.x * b.z,
      z: a.x * b.y - a.y * b.x
    };
  },
  len(v) { return Math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z); },
  normalize(v) {
    const l = Vec3.len(v);
    return l > 1e-9 ? Vec3.scale(v, 1 / l) : { x: 0, y: 0, z: 0 };
  },
  lerp(a, b, t) {
    return {
      x: a.x + (b.x - a.x) * t,
      y: a.y + (b.y - a.y) * t,
      z: a.z + (b.z - a.z) * t
    };
  }
};
// =============================================================================
// Section 3: MAT4 MATH
// =============================================================================
// Row-major flat array of 16 elements.
// Row i, column j => index i*4+j.
// transformPoint applies m * [x,y,z,w]^T (standard matrix-vector multiply).
const Mat4 = {
  identity() {
    return [
      1, 0, 0, 0,
      0, 1, 0, 0,
      0, 0, 1, 0,
      0, 0, 0, 1
    ];
  },
  multiply(a, b) {
    const r = new Array(16).fill(0);
    for (let i = 0; i < 4; i++) {
      for (let j = 0; j < 4; j++) {
        for (let k = 0; k < 4; k++) {
          r[i * 4 + j] += a[i * 4 + k] * b[k * 4 + j];
        }
      }
    }
    return r;
  },
  lookAt(eye, center, up) {
    const f = Vec3.normalize(Vec3.sub(center, eye));
    const r = Vec3.normalize(Vec3.cross(f, up));
    const u = Vec3.cross(r, f);
    return [
      r.x,  r.y,  r.z,  -Vec3.dot(r, eye),
      u.x,  u.y,  u.z,  -Vec3.dot(u, eye),
      -f.x, -f.y, -f.z,  Vec3.dot(f, eye),
      0,    0,    0,     1
    ];
  },
  perspective(fovY, aspect, near, far) {
    const f = 1 / Math.tan(fovY / 2);
    return [
      f / aspect, 0, 0,                             0,
      0,          f, 0,                             0,
      0,          0, (far + near) / (near - far),   2 * far * near / (near - far),
      0,          0, -1,                            0
    ];
  },
  transformPoint(m, p) {
    const x = p.x, y = p.y, z = p.z, w = (p.w !== undefined) ? p.w : 1;
    return {
      x: m[0]*x  + m[1]*y  + m[2]*z  + m[3]*w,
      y: m[4]*x  + m[5]*y  + m[6]*z  + m[7]*w,
      z: m[8]*x  + m[9]*y  + m[10]*z + m[11]*w,
      w: m[12]*x + m[13]*y + m[14]*z + m[15]*w
    };
  },
  fromRotationY(a) {
    const c = Math.cos(a), s = Math.sin(a);
    return [
      c,  0, s, 0,
      0,  1, 0, 0,
      -s, 0, c, 0,
      0,  0, 0, 1
    ];
  },
  fromRotationX(a) {
    const c = Math.cos(a), s = Math.sin(a);
    return [
      1,  0,  0, 0,
      0,  c, -s, 0,
      0,  s,  c, 0,
      0,  0,  0, 1
    ];
  }
};
// =============================================================================
// Section 4: CAMERA
// =============================================================================
class Camera {
  constructor() {
    this.theta  = Math.PI / 4;
    this.phi    = Math.PI / 3;
    this.radius = 200;
    this.target = { x: 0, y: 0, z: 0 };
    this.fov    = 50 * Math.PI / 180;
    this.aspect = 1;
  }
  get position() {
    const sp = Math.sin(this.phi);
    const cp = Math.cos(this.phi);
    const st = Math.sin(this.theta);
    const ct = Math.cos(this.theta);
    return {
      x: this.target.x + this.radius * sp * ct,
      y: this.target.y + this.radius * cp,
      z: this.target.z + this.radius * sp * st
    };
  }
  get viewMatrix() {
    return Mat4.lookAt(this.position, this.target, { x: 0, y: 1, z: 0 });
  }
  get projMatrix() {
    return Mat4.perspective(this.fov, this.aspect, 0.1, 2000);
  }
  orbit(dTheta, dPhi) {
    this.theta += dTheta;
    this.phi    = Math.max(0.05, Math.min(Math.PI - 0.05, this.phi + dPhi));
  }
  zoom(delta) {
    this.radius = Math.max(10, Math.min(2000, this.radius + delta));
  }
  setAspect(w, h) {
    this.aspect = h > 0 ? w / h : 1;
  }
  setPreset(preset, customTheta = 0, customPhi = Math.PI / 2) {
    switch (preset) {
      case EXPORT_PRESET.SIDE_2D:
        this.theta = 0;
        this.phi   = Math.PI / 2;
        break;
      case EXPORT_PRESET.ISO_4:
      case EXPORT_PRESET.ISO_6:
      case EXPORT_PRESET.ISO_8:
      case EXPORT_PRESET.ISO_8:
        this.phi = Math.PI / 4;
        break;
      case EXPORT_PRESET.CUSTOM:
        this.theta = customTheta;
        this.phi   = customPhi;
        break;
    }
  }
  worldToScreen(point, canvasW, canvasH) {
    const vp   = Mat4.multiply(this.projMatrix, this.viewMatrix);
    const clip = Mat4.transformPoint(vp, { x: point.x, y: point.y, z: point.z, w: 1 });
    if (clip.w <= 0) return null;
    const nx = clip.x / clip.w;
    const ny = clip.y / clip.w;
    return {
      x:     (nx + 1) * 0.5 * canvasW,
      y:     (1 - ny) * 0.5 * canvasH,
      depth:  clip.z  / clip.w
    };
  }
  screenToWorldOnPlane(sx, sy, planeY, canvasW, canvasH) {
    const ndcX    = (sx / canvasW) * 2 - 1;
    const ndcY    = 1 - (sy / canvasH) * 2;
    const tanHalf = Math.tan(this.fov / 2);
    const rx = ndcX * tanHalf * this.aspect;
    const ry = ndcY * tanHalf;
    const rz = -1;
    // Transform view-space direction to world space via transpose of rotation block
    const vm  = this.viewMatrix;
    const rwx = vm[0] * rx + vm[4] * ry + vm[8]  * rz;
    const rwy = vm[1] * rx + vm[5] * ry + vm[9]  * rz;
    const rwz = vm[2] * rx + vm[6] * ry + vm[10] * rz;
    const eye = this.position;
    if (Math.abs(rwy) < 1e-6) {
      // Ray parallel to plane — return a distant point in the ray direction
      return { x: eye.x + rwx * PARALLEL_RAY_DISTANCE, y: planeY, z: eye.z + rwz * PARALLEL_RAY_DISTANCE };
    }
    const t = (planeY - eye.y) / rwy;
    return { x: eye.x + rwx * t, y: planeY, z: eye.z + rwz * t };
  }
  // Camera right/up from view matrix rows 0 and 1 (first 3 elements)
  getRightVector() {
    const vm = this.viewMatrix;
    return { x: vm[0], y: vm[1], z: vm[2] };
  }
  getUpVector() {
    const vm = this.viewMatrix;
    return { x: vm[4], y: vm[5], z: vm[6] };
  }
}
// =============================================================================
// Section 5: SCENE
// =============================================================================
class Scene {
  constructor() {
    this.nodes       = new Map();
    this.edges       = new Map();
    this._nextNodeId = 1;
    this._nextEdgeId = 1;
  }
  addNode(x = 0, y = 0, z = 0, name = null) {
    const id   = this._nextNodeId++;
    const node = { id, x, y, z, name: name || `Node ${id}` };
    this.nodes.set(id, node);
    return node;
  }
  removeNode(id) {
    this.nodes.delete(id);
    for (const [eid, edge] of this.edges) {
      if (edge.from === id || edge.to === id) this.edges.delete(eid);
    }
  }
  addEdge(fromId, toId, type = EDGE.DIRECTED) {
    if (fromId === toId) return null;
    for (const edge of this.edges.values()) {
      const sameDir  = edge.from === fromId && edge.to === toId;
      const revDir   = edge.from === toId   && edge.to === fromId;
      if (edge.type === EDGE.BIDIRECTIONAL && (sameDir || revDir)) return null;
      if (sameDir && edge.type === type) return null;
    }
    const id   = this._nextEdgeId++;
    const edge = { id, from: fromId, to: toId, type };
    this.edges.set(id, edge);
    return edge;
  }
  removeEdge(id) {
    this.edges.delete(id);
  }
  updateNodePosition(id, x, y, z) {
    const node = this.nodes.get(id);
    if (node) { node.x = x; node.y = y; node.z = z; }
  }
  getNode(id)  { return this.nodes.get(id); }
  getEdge(id)  { return this.edges.get(id); }
  getEdgesForNode(nodeId) {
    return Array.from(this.edges.values()).filter(
      e => e.from === nodeId || e.to === nodeId
    );
  }
  // BFS: DIRECTED edges propagate; BIDIRECTIONAL adds neighbour (no recurse); ELASTIC skipped.
  getDragSet(startNodeId) {
    const result   = new Set([startNodeId]);
    const bfsQueue = [startNodeId];
    while (bfsQueue.length > 0) {
      const nodeId = bfsQueue.shift();
      for (const edge of this.getEdgesForNode(nodeId)) {
        if (edge.type === EDGE.DIRECTED && edge.from === nodeId) {
          if (!result.has(edge.to)) {
            result.add(edge.to);
            bfsQueue.push(edge.to);
          }
        } else if (edge.type === EDGE.BIDIRECTIONAL) {
          const other = edge.from === nodeId ? edge.to : edge.from;
          result.add(other); // no recursion
        }
        // ELASTIC: skip
      }
    }
    return result;
  }
  clone() {
    const nodes = new Map();
    for (const [id, n] of this.nodes) nodes.set(id, { ...n });
    const edges = new Map();
    for (const [id, e] of this.edges) edges.set(id, { ...e });
    return { nodes, edges };
  }
  getSnapshot() {
    return {
      nodes: Array.from(this.nodes.values()).map(n => ({ ...n })),
      edges: Array.from(this.edges.values()).map(e => ({ ...e }))
    };
  }
  applySnapshot(snapshot) {
    this.nodes.clear();
    this.edges.clear();
    this._nextNodeId = 1;
    this._nextEdgeId = 1;
    for (const n of snapshot.nodes) {
      this.nodes.set(n.id, { ...n });
      if (n.id >= this._nextNodeId) this._nextNodeId = n.id + 1;
    }
    for (const e of snapshot.edges) {
      this.edges.set(e.id, { ...e });
      if (e.id >= this._nextEdgeId) this._nextEdgeId = e.id + 1;
    }
  }
  toJSON()       { return this.getSnapshot(); }
  fromJSON(data) { this.applySnapshot(data);  }
}
// =============================================================================
// Section 6: RENDERER
// =============================================================================
class Renderer {
  constructor(canvas, scene, camera) {
    this.canvas      = canvas;
    this.ctx         = canvas.getContext('2d');
    this.scene       = scene;
    this.camera      = camera;
    this._canvasSize = 64;
    this._c = {
      bg:              '#1a1a2e',
      grid:            '#2a2a4a',
      gridBounds:      '#6666aa',
      boundsBox:       '#5577bb',
      boundsLabel:     '#8899cc',
      nodeDefault:     '#7ec8e3',
      nodeSelected:    '#ffd700',
      nodeHover:       '#b0e0ff',
      nodeStroke:      '#ffffff',
      edgeDirected:    '#78c878',
      edgeBidirect:    '#c878c8',
      edgeElastic:     '#c8a878',
      edgeSelected:    '#ffd700',
      axisX:           '#e05555',
      axisY:           '#55e055',
      axisZ:           '#5588ee',
      label:           '#c0c0d0'
    };
  }
  setCanvasSize(size) { this._canvasSize = size; }
  render(selectedNodeId, selectedEdgeId, activeTool, pendingEdgeFromId, mouseScreenPos) {
    const W = this.canvas.width;
    const H = this.canvas.height;
    this._clear(W, H);
    this._drawGrid(this._canvasSize);
    this._drawRenderBounds(this._canvasSize);
    this._drawAxes();
    for (const item of this._sortRenderables()) {
      if (item.type === 'edge') {
        this._drawEdge(item.data, item.data.id === selectedEdgeId);
      } else {
        const sp = this._projectNode(item.data);
        if (sp) this._drawNode(item.data, item.data.id === selectedNodeId, sp);
      }
    }
    if (pendingEdgeFromId !== null && mouseScreenPos) {
      const fn = this.scene.getNode(pendingEdgeFromId);
      if (fn) this._drawPendingEdge(fn, mouseScreenPos);
    }
  }
  _clear(W, H) {
    this.ctx.fillStyle = this._c.bg;
    this.ctx.fillRect(0, 0, W, H);
  }
  _drawGrid(canvasSize) {
    const ctx  = this.ctx;
    const W    = this.canvas.width;
    const H    = this.canvas.height;
    const hs   = canvasSize / 2;
    const step = Math.max(4, canvasSize / 8);
    const ext  = hs + step * 2; // extend grid slightly beyond bounds
    ctx.strokeStyle = this._c.grid;
    ctx.lineWidth   = 0.5;
    for (let i = -ext; i <= ext; i += step) {
      this._drawLine2D(ctx,
        this.camera.worldToScreen({ x: i,    y: 0, z: -ext }, W, H),
        this.camera.worldToScreen({ x: i,    y: 0, z:  ext }, W, H)
      );
      this._drawLine2D(ctx,
        this.camera.worldToScreen({ x: -ext, y: 0, z: i    }, W, H),
        this.camera.worldToScreen({ x:  ext, y: 0, z: i    }, W, H)
      );
    }
    // Target bounds rectangle
    const corners = [
      { x: -hs, y: 0, z: -hs }, { x: hs, y: 0, z: -hs },
      { x:  hs, y: 0, z:  hs }, { x: -hs, y: 0, z: hs }
    ].map(c => this.camera.worldToScreen(c, W, H)).filter(Boolean);
    if (corners.length === 4) {
      ctx.strokeStyle = this._c.gridBounds;
      ctx.lineWidth   = 2;
      ctx.beginPath();
      ctx.moveTo(corners[0].x, corners[0].y);
      for (let i = 1; i < 4; i++) ctx.lineTo(corners[i].x, corners[i].y);
      ctx.closePath();
      ctx.stroke();
    }
    // Center cross lines on the floor plane (X=0 and Z=0 axes)
    ctx.lineWidth   = 1;
    ctx.setLineDash([6, 4]);
    ctx.globalAlpha = 0.45;
    ctx.strokeStyle = this._c.axisX;
    this._drawLine2D(ctx,
      this.camera.worldToScreen({ x: -hs, y: 0, z: 0 }, W, H),
      this.camera.worldToScreen({ x:  hs, y: 0, z: 0 }, W, H)
    );
    ctx.strokeStyle = this._c.axisZ;
    this._drawLine2D(ctx,
      this.camera.worldToScreen({ x: 0, y: 0, z: -hs }, W, H),
      this.camera.worldToScreen({ x: 0, y: 0, z:  hs }, W, H)
    );
    ctx.setLineDash([]);
    ctx.globalAlpha = 1;
  }
  _drawLine2D(ctx, a, b) {
    if (!a || !b) return;
    ctx.beginPath();
    ctx.moveTo(a.x, a.y);
    ctx.lineTo(b.x, b.y);
    ctx.stroke();
  }
  _drawAxes() {
    const W  = this.canvas.width;
    const H  = this.canvas.height;
    const o  = this.camera.worldToScreen({ x: 0, y: 0, z: 0 }, W, H);
    if (!o) return;
    const len  = 10;
    const axes = [
      { end: { x: len, y: 0,   z: 0   }, color: this._c.axisX, label: 'X' },
      { end: { x: 0,   y: len, z: 0   }, color: this._c.axisY, label: 'Y' },
      { end: { x: 0,   y: 0,   z: len }, color: this._c.axisZ, label: 'Z' }
    ];
    const ctx = this.ctx;
    ctx.font      = 'bold 10px sans-serif';
    ctx.textAlign = 'center';
    for (const { end, color, label } of axes) {
      const ep = this.camera.worldToScreen(end, W, H);
      if (!ep) continue;
      ctx.strokeStyle = color;
      ctx.lineWidth   = 2;
      ctx.beginPath();
      ctx.moveTo(o.x, o.y);
      ctx.lineTo(ep.x, ep.y);
      ctx.stroke();
      const dx  = ep.x - o.x;
      const dy  = ep.y - o.y;
      const labelOffset = Math.hypot(dx, dy);
      if (labelOffset > 0) {
        ctx.fillStyle = color;
        ctx.fillText(label, ep.x + (dx / labelOffset) * 9, ep.y + (dy / labelOffset) * 9 + 3);
      }
    }
  }
  _drawRenderBounds(canvasSize) {
    const ctx = this.ctx;
    const W   = this.canvas.width;
    const H   = this.canvas.height;
    const hs  = canvasSize / 2;
    // Project all 8 corners of the bounding cube
    const b = [
      this.camera.worldToScreen({ x: -hs, y: -hs, z: -hs }, W, H),
      this.camera.worldToScreen({ x:  hs, y: -hs, z: -hs }, W, H),
      this.camera.worldToScreen({ x:  hs, y: -hs, z:  hs }, W, H),
      this.camera.worldToScreen({ x: -hs, y: -hs, z:  hs }, W, H)
    ];
    const t = [
      this.camera.worldToScreen({ x: -hs, y:  hs, z: -hs }, W, H),
      this.camera.worldToScreen({ x:  hs, y:  hs, z: -hs }, W, H),
      this.camera.worldToScreen({ x:  hs, y:  hs, z:  hs }, W, H),
      this.camera.worldToScreen({ x: -hs, y:  hs, z:  hs }, W, H)
    ];
    ctx.save();
    ctx.strokeStyle = this._c.boundsBox;
    ctx.lineWidth   = 1;
    ctx.setLineDash([5, 4]);
    ctx.globalAlpha = 0.6;
    // Bottom face
    if (b.every(Boolean)) {
      ctx.beginPath();
      ctx.moveTo(b[0].x, b[0].y);
      for (let i = 1; i < 4; i++) ctx.lineTo(b[i].x, b[i].y);
      ctx.closePath();
      ctx.stroke();
    }
    // Top face
    if (t.every(Boolean)) {
      ctx.beginPath();
      ctx.moveTo(t[0].x, t[0].y);
      for (let i = 1; i < 4; i++) ctx.lineTo(t[i].x, t[i].y);
      ctx.closePath();
      ctx.stroke();
    }
    // Vertical edges connecting bottom to top
    for (let i = 0; i < 4; i++) {
      if (b[i] && t[i]) this._drawLine2D(ctx, b[i], t[i]);
    }
    ctx.restore();
    // "Export bounds" label above the top face
    const topValid = t.filter(Boolean);
    if (topValid.length > 0) {
      const mx = topValid.reduce((s, p) => s + p.x, 0) / topValid.length;
      const my = topValid.reduce((mn, p) => Math.min(mn, p.y), Infinity) - 6;
      ctx.save();
      ctx.fillStyle  = this._c.boundsLabel;
      ctx.font       = '10px sans-serif';
      ctx.textAlign  = 'center';
      ctx.fillText(`Export bounds (${canvasSize}×${canvasSize})`, mx, my);
      ctx.restore();
    }
  }
  _sortRenderables() {
    const W     = this.canvas.width;
    const H     = this.canvas.height;
    const items = [];
    for (const node of this.scene.nodes.values()) {
      const sp = this.camera.worldToScreen({ x: node.x, y: node.y, z: node.z }, W, H);
      items.push({ type: 'node', data: node, depth: sp ? sp.depth : -Infinity });
    }
    for (const edge of this.scene.edges.values()) {
      const f  = this.scene.getNode(edge.from);
      const t  = this.scene.getNode(edge.to);
      if (!f || !t) continue;
      const sA = this.camera.worldToScreen({ x: f.x, y: f.y, z: f.z }, W, H);
      const sB = this.camera.worldToScreen({ x: t.x, y: t.y, z: t.z }, W, H);
      const d  = ((sA ? sA.depth : 0) + (sB ? sB.depth : 0)) / 2;
      items.push({ type: 'edge', data: edge, depth: d });
    }
    items.sort((a, b) => b.depth - a.depth); // back-to-front
    return items;
  }
  _projectNode(node) {
    return this.camera.worldToScreen(
      { x: node.x, y: node.y, z: node.z },
      this.canvas.width, this.canvas.height
    );
  }
  _drawEdge(edge, isSelected) {
    const from = this.scene.getNode(edge.from);
    const to   = this.scene.getNode(edge.to);
    if (!from || !to) return;
    const W   = this.canvas.width;
    const H   = this.canvas.height;
    const spA = this.camera.worldToScreen({ x: from.x, y: from.y, z: from.z }, W, H);
    const spB = this.camera.worldToScreen({ x: to.x,   y: to.y,   z: to.z   }, W, H);
    if (!spA || !spB) return;
    const colorMap = {
      [EDGE.DIRECTED]:      this._c.edgeDirected,
      [EDGE.BIDIRECTIONAL]: this._c.edgeBidirect,
      [EDGE.ELASTIC]:       this._c.edgeElastic
    };
    const color = isSelected ? this._c.edgeSelected : (colorMap[edge.type] || this._c.edgeDirected);
    const ctx   = this.ctx;
    ctx.save();
    ctx.strokeStyle = color;
    ctx.lineWidth   = isSelected ? 2.5 : 1.5;
    if (edge.type === EDGE.ELASTIC) ctx.setLineDash([5, 5]);
    ctx.beginPath();
    ctx.moveTo(spA.x, spA.y);
    ctx.lineTo(spB.x, spB.y);
    ctx.stroke();
    ctx.setLineDash([]);
    if (edge.type === EDGE.DIRECTED)      this._drawArrowHead(ctx, spA, spB, color);
    if (edge.type === EDGE.BIDIRECTIONAL) {
      this._drawArrowHead(ctx, spA, spB, color);
      this._drawArrowHead(ctx, spB, spA, color);
    }
    ctx.restore();
  }
  _drawArrowHead(ctx, from2d, to2d, color) {
    const dx  = to2d.x - from2d.x;
    const dy  = to2d.y - from2d.y;
    const len = Math.hypot(dx, dy);
    if (len < 1) return;
    const nx   = dx / len;
    const ny   = dy / len;
    const size = 10;
    const tipX = to2d.x - nx * NODE_RADIUS;
    const tipY = to2d.y - ny * NODE_RADIUS;
    ctx.save();
    ctx.fillStyle = color;
    ctx.beginPath();
    ctx.moveTo(tipX, tipY);
    ctx.lineTo(tipX - nx * size + ny * size * 0.4, tipY - ny * size - nx * size * 0.4);
    ctx.lineTo(tipX - nx * size - ny * size * 0.4, tipY - ny * size + nx * size * 0.4);
    ctx.closePath();
    ctx.fill();
    ctx.restore();
  }
  _drawNode(node, isSelected, sp) {
    const ctx = this.ctx;
    const r   = NODE_RADIUS;
    ctx.save();
    ctx.beginPath();
    ctx.arc(sp.x, sp.y, r, 0, Math.PI * 2);
    ctx.fillStyle   = isSelected ? this._c.nodeSelected : this._c.nodeDefault;
    ctx.fill();
    ctx.strokeStyle = this._c.nodeStroke;
    ctx.lineWidth   = 1.5;
    ctx.stroke();
    ctx.fillStyle  = this._c.label;
    ctx.font       = '10px sans-serif';
    ctx.textAlign  = 'center';
    ctx.fillText(node.name, sp.x, sp.y + r + 12);
    ctx.restore();
  }
  _drawPendingEdge(fromNode, mousePos) {
    const W  = this.canvas.width;
    const H  = this.canvas.height;
    const sp = this.camera.worldToScreen({ x: fromNode.x, y: fromNode.y, z: fromNode.z }, W, H);
    if (!sp) return;
    const ctx = this.ctx;
    ctx.save();
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth   = 1.5;
    ctx.setLineDash([5, 5]);
    ctx.beginPath();
    ctx.moveTo(sp.x, sp.y);
    ctx.lineTo(mousePos.x, mousePos.y);
    ctx.stroke();
    ctx.setLineDash([]);
    ctx.restore();
  }
}
// =============================================================================
// Section 7: INPUT HANDLER
// =============================================================================
class InputHandler {
  constructor(canvas, app) {
    this.canvas  = canvas;
    this.app     = app;
    this._isDragging      = false;
    this._isOrbit         = false;
    this._isNodeDrag      = false;
    this._dragNodeId      = null;
    this._dragSet         = null;
    this._lastPointer     = null;
    this._touchStartDist  = null;
    this._pointerDownPos  = null;
    this._bind();
  }
  _bind() {
    const c = this.canvas;
    c.addEventListener('pointerdown',   e => this._onPointerDown(e));
    c.addEventListener('pointermove',   e => this._onPointerMove(e));
    c.addEventListener('pointerup',     e => this._onPointerUp(e));
    c.addEventListener('pointercancel', e => this._onPointerUp(e));
    c.addEventListener('wheel',         e => this._onWheel(e), { passive: false });
    c.addEventListener('contextmenu',   e => e.preventDefault());
    c.addEventListener('touchstart',    e => this._onTouchStart(e), { passive: false });
    c.addEventListener('touchmove',     e => this._onTouchMove(e),  { passive: false });
    c.addEventListener('touchend',      e => this._onTouchEnd(e),   { passive: false });
    document.addEventListener('keydown', e => this._onKeyDown(e));
  }
  _getEventPos(e) {
    const rect   = this.canvas.getBoundingClientRect();
    const scaleX = this.canvas.width  / rect.width;
    const scaleY = this.canvas.height / rect.height;
    return {
      x: (e.clientX - rect.left) * scaleX,
      y: (e.clientY - rect.top)  * scaleY
    };
  }
  _onPointerDown(e) {
    if (e.pointerType === 'touch' && !e.isPrimary) return;
    this.canvas.setPointerCapture(e.pointerId);
    const pos     = this._getEventPos(e);
    this._lastPointer    = pos;
    this._pointerDownPos = pos;
    this._isDragging     = false;
    const nodeId  = this._hitTestNode(pos.x, pos.y);
    switch (this.app.activeTool) {
      case TOOL.SELECT:    this._handleSelectToolDown(pos.x, pos.y, nodeId); break;
      case TOOL.ADD_NODE:  this._handleAddNodeDown(pos.x, pos.y);            break;
      case TOOL.ADD_EDGE:  this._handleAddEdgeDown(pos.x, pos.y, nodeId);    break;
      case TOOL.DELETE:    this._handleDeleteDown(pos.x, pos.y, nodeId);     break;
    }
  }
  _onPointerMove(e) {
    if (e.pointerType === 'touch' && !e.isPrimary) return;
    const pos = this._getEventPos(e);
    this.app.setMousePos(pos);
    if (!this._lastPointer) return;
    const dx = pos.x - this._lastPointer.x;
    const dy = pos.y - this._lastPointer.y;
    if (Math.abs(dx) > 1 || Math.abs(dy) > 1) this._isDragging = true;
    if (this._isOrbit) {
      this.app.camera.orbit(dx * 0.005, dy * 0.005);
    } else if (this._isNodeDrag && this._dragSet) {
      this._moveNodesByScreenDelta(dx, dy);
    }
    this._lastPointer = pos;
  }
  _onPointerUp(e) {
    if (e.pointerType === 'touch' && !e.isPrimary) return;
    if (this._isNodeDrag) this.app.animController.captureCurrentFrame();
    this._isOrbit    = false;
    this._isNodeDrag = false;
    this._dragSet    = null;
    this._dragNodeId = null;
    this._lastPointer = null;
  }
  _onWheel(e) {
    e.preventDefault();
    this.app.camera.zoom(e.deltaY * 0.5);
  }
  _onTouchStart(e) {
    if (e.touches.length === 2) {
      e.preventDefault();
      const dx = e.touches[0].clientX - e.touches[1].clientX;
      const dy = e.touches[0].clientY - e.touches[1].clientY;
      this._touchStartDist = Math.hypot(dx, dy);
    }
  }
  _onTouchMove(e) {
    if (e.touches.length === 2) {
      e.preventDefault();
      const dx   = e.touches[0].clientX - e.touches[1].clientX;
      const dy   = e.touches[0].clientY - e.touches[1].clientY;
      const dist = Math.hypot(dx, dy);
      if (this._touchStartDist) {
        this.app.camera.zoom((this._touchStartDist - dist) * 0.5);
        this._touchStartDist = dist;
      }
    }
  }
  _onTouchEnd(e) {
    if (e.touches.length < 2) this._touchStartDist = null;
  }
  _onKeyDown(e) {
    const tag = document.activeElement.tagName;
    if (tag === 'INPUT' || tag === 'SELECT' || tag === 'TEXTAREA') return;
    switch (e.key) {
      case 'v': case 'V':                this.app.setTool(TOOL.SELECT);   break;
      case 'n': case 'N':                this.app.setTool(TOOL.ADD_NODE); break;
      case 'e': case 'E':                this.app.setTool(TOOL.ADD_EDGE); break;
      case 'Delete': case 'Backspace':   this.app.deleteSelected();        break;
      case ' ':
        e.preventDefault();
        this.app.animController.isPlaying ? this.app.stopPlayback() : this.app.startPlayback();
        break;
      case 'ArrowLeft':  this.app.prevFrame(); break;
      case 'ArrowRight': this.app.nextFrame(); break;
    }
  }
  _hitTestNode(sx, sy) {
    const W = this.canvas.width;
    const H = this.canvas.height;
    let closest = null;
    let closestD2 = HIT_RADIUS * HIT_RADIUS;
    for (const node of this.app.scene.nodes.values()) {
      const sp = this.app.camera.worldToScreen({ x: node.x, y: node.y, z: node.z }, W, H);
      if (!sp) continue;
      const d2 = (sp.x - sx) ** 2 + (sp.y - sy) ** 2;
      if (d2 < closestD2) { closestD2 = d2; closest = node.id; }
    }
    return closest;
  }
  _hitTestEdge(sx, sy) {
    const W = this.canvas.width;
    const H = this.canvas.height;
    for (const edge of this.app.scene.edges.values()) {
      const f  = this.app.scene.getNode(edge.from);
      const t  = this.app.scene.getNode(edge.to);
      if (!f || !t) continue;
      const sA = this.app.camera.worldToScreen({ x: f.x, y: f.y, z: f.z }, W, H);
      const sB = this.app.camera.worldToScreen({ x: t.x, y: t.y, z: t.z }, W, H);
      if (!sA || !sB) continue;
      if (this._ptSegDist(sx, sy, sA.x, sA.y, sB.x, sB.y) <= EDGE_HIT_WIDTH) return edge.id;
    }
    return null;
  }
  _ptSegDist(px, py, ax, ay, bx, by) {
    const dx = bx - ax, dy = by - ay;
    const l2 = dx * dx + dy * dy;
    if (l2 === 0) return Math.hypot(px - ax, py - ay);
    const t  = Math.max(0, Math.min(1, ((px - ax) * dx + (py - ay) * dy) / l2));
    return Math.hypot(px - (ax + t * dx), py - (ay + t * dy));
  }
  _handleSelectToolDown(sx, sy, nodeId) {
    if (nodeId !== null) {
      this.app.selectNode(nodeId);
      this._isNodeDrag = true;
      this._dragNodeId = nodeId;
      this._dragSet    = this.app.scene.getDragSet(nodeId);
    } else {
      const edgeId = this._hitTestEdge(sx, sy);
      if (edgeId !== null) {
        this.app.selectEdge(edgeId);
      } else {
        this.app.deselect();
        this._isOrbit = true;
      }
    }
  }
  _handleAddNodeDown(sx, sy) {
    const pos = this.app.camera.screenToWorldOnPlane(sx, sy, 0, this.canvas.width, this.canvas.height);
    this.app.addNode(pos.x, pos.y, pos.z);
  }
  _handleAddEdgeDown(sx, sy, nodeId) {
    if (nodeId === null) { this.app.cancelPendingEdge(); return; }
    if (this.app.pendingEdgeFromId === null) {
      this.app.setPendingEdgeFrom(nodeId);
    } else {
      if (nodeId !== this.app.pendingEdgeFromId) {
        this.app.addEdge(this.app.pendingEdgeFromId, nodeId, this.app.activeEdgeType);
      }
      this.app.cancelPendingEdge();
    }
  }
  _handleDeleteDown(sx, sy, nodeId) {
    if (nodeId !== null) {
      this.app.scene.removeNode(nodeId);
      this.app.deselect();
      this.app.animController.captureCurrentFrame();
    } else {
      const edgeId = this._hitTestEdge(sx, sy);
      if (edgeId !== null) {
        this.app.scene.removeEdge(edgeId);
        this.app.deselect();
      }
    }
  }
  _moveNodesByScreenDelta(dx, dy) {
    const camera = this.app.camera;
    const node   = this.app.scene.getNode(this._dragNodeId);
    if (!node) return;
    const dist  = Vec3.len(Vec3.sub({ x: node.x, y: node.y, z: node.z }, camera.position));
    const scale = (2 * Math.tan(camera.fov / 2) * Math.max(dist, 0.1)) / this.canvas.height;
    const right      = camera.getRightVector();
    const up         = camera.getUpVector();
    const worldDelta = Vec3.add(
      Vec3.scale(right, dx * scale),
      Vec3.scale(up, -dy * scale)
    );
    for (const nid of this._dragSet) {
      const n = this.app.scene.getNode(nid);
      if (n) this.app.scene.updateNodePosition(nid, n.x + worldDelta.x, n.y + worldDelta.y, n.z + worldDelta.z);
    }
  }
}
// =============================================================================
// Section 8: ANIMATION CONTROLLER
// =============================================================================
class AnimationController {
  constructor(scene) {
    this._scene        = scene;
    this._frames       = [];
    this._currentIndex = 0;
    this._playInterval = null;
    this._nextFrameId  = 1;
    this.addFrame(); // start with one frame
  }
  get frameCount()        { return this._frames.length; }
  get currentFrameIndex() { return this._currentIndex;  }
  get isPlaying()         { return this._playInterval !== null; }
  addFrame() {
    const frame = { id: this._nextFrameId++, positions: this._capturePositions() };
    this._frames.push(frame);
    this._currentIndex = this._frames.length - 1;
    return frame;
  }
  removeFrame(index) {
    if (this._frames.length <= 1) return;
    this._frames.splice(index, 1);
    this._currentIndex = Math.min(this._currentIndex, this._frames.length - 1);
    this._applyFrame(this._frames[this._currentIndex]);
  }
  duplicateFrame(index) {
    const src   = this._frames[index];
    if (!src) return;
    const frame = { id: this._nextFrameId++, positions: JSON.parse(JSON.stringify(src.positions)) };
    this._frames.splice(index + 1, 0, frame);
    this._currentIndex = index + 1;
  }
  setFrame(index) {
    if (index < 0 || index >= this._frames.length) return;
    this._currentIndex = index;
    this._applyFrame(this._frames[index]);
  }
  captureCurrentFrame() {
    if (this._frames.length === 0) return;
    this._frames[this._currentIndex].positions = this._capturePositions();
  }
  play(fps, onTick) {
    this.stop();
    const ms = 1000 / Math.max(1, fps);
    this._playInterval = setInterval(() => {
      const next = (this._currentIndex + 1) % this._frames.length;
      this.setFrame(next);
      onTick(next);
    }, ms);
  }
  stop() {
    if (this._playInterval) { clearInterval(this._playInterval); this._playInterval = null; }
  }
  getFrameSnapshot(index) {
    return this._frames[index] ? { ...this._frames[index].positions } : {};
  }
  _capturePositions() {
    const positions = {};
    for (const [id, node] of this._scene.nodes) {
      positions[id] = { x: node.x, y: node.y, z: node.z };
    }
    return positions;
  }
  _applyFrame(frame) {
    if (!frame) return;
    for (const [id] of this._scene.nodes) {
      const pos = frame.positions[id];
      if (pos) this._scene.updateNodePosition(id, pos.x, pos.y, pos.z);
    }
  }
  toJSON() {
    return {
      frames:       this._frames.map(f => ({ id: f.id, positions: { ...f.positions } })),
      currentIndex: this._currentIndex
    };
  }
  fromJSON(data, scene) {
    this._scene        = scene;
    this._frames       = data.frames.map(f => ({ id: f.id, positions: { ...f.positions } }));
    this._nextFrameId  = Math.max(0, ...this._frames.map(f => f.id)) + 1;
    this._currentIndex = Math.min(data.currentIndex || 0, Math.max(0, this._frames.length - 1));
    if (this._frames.length === 0) { this.addFrame(); } else { this.setFrame(this._currentIndex); }
  }
}
// =============================================================================
// Section 9: EXPORT MANAGER
// =============================================================================
class ExportManager {
  async render(scene, animController, preset, customAngles, canvasSize, nodeRadius, onProgress) {
    const angles     = this._getPresetAngles(preset, customAngles);
    const frameCount = animController.frameCount;
    const cellSize   = canvasSize;
    const sheet = document.createElement('canvas');
    sheet.width  = frameCount * cellSize;
    sheet.height = angles.length * cellSize;
    const sheetCtx = sheet.getContext('2d');
    const cell    = document.createElement('canvas');
    cell.width    = cellSize;
    cell.height   = cellSize;
    const cellCtx = cell.getContext('2d');
    const cam = new Camera();
    cam.setAspect(cellSize, cellSize);
    cam.target = { x: 0, y: 0, z: 0 };
    cam.radius = this._computeExportRadius(scene, canvasSize);
    cam.fov    = 50 * Math.PI / 180;
    let done = 0;
    const total = angles.length * frameCount;
    for (let di = 0; di < angles.length; di++) {
      cam.theta = angles[di].theta;
      cam.phi   = angles[di].phi;
      for (let fi = 0; fi < frameCount; fi++) {
        const snap = animController.getFrameSnapshot(fi);
        this._renderFrameToCanvas(cellCtx, scene, snap, cam, cellSize, nodeRadius);
        sheetCtx.drawImage(cell, fi * cellSize, di * cellSize);
        done++;
        if (onProgress) onProgress(done / total);
      }
    }
    return sheet.toDataURL('image/png');
  }
  _computeExportRadius(scene, canvasSize) {
    if (scene.nodes.size === 0) return canvasSize;
    let maxDist = 0;
    for (const n of scene.nodes.values()) {
      maxDist = Math.max(maxDist, Math.hypot(n.x, n.y, n.z));
    }
    return Math.max(maxDist * 2.5, canvasSize);
  }
  _getPresetAngles(preset, customAngles) {
    const PI = Math.PI;
    switch (preset) {
      case EXPORT_PRESET.SIDE_2D:
        return [{ theta: 0,       phi: PI / 2, label: 'Side' }];
      case EXPORT_PRESET.ISO_4:
        return [0, PI/2, PI, 3*PI/2].map((t, i) => ({ theta: t, phi: PI/4, label: `Dir${i}` }));
      case EXPORT_PRESET.ISO_6:
        return [0, PI/3, 2*PI/3, PI, 4*PI/3, 5*PI/3].map((t, i) => ({ theta: t, phi: PI/4, label: `Dir${i}` }));
      case EXPORT_PRESET.ISO_8:
        return Array.from({ length: 8 }, (_, i) => ({ theta: i * PI / 4, phi: PI / 4, label: `Dir${i}` }));
      case EXPORT_PRESET.CUSTOM:
        return customAngles && customAngles.length ? customAngles : [{ theta: 0, phi: PI/2, label: 'Custom' }];
      default:
        return [{ theta: 0, phi: PI / 2, label: 'Side' }];
    }
  }
  _renderFrameToCanvas(ctx, scene, frameSnapshot, camera, cellSize, nodeRadius) {
    ctx.clearRect(0, 0, cellSize, cellSize);
    const r = nodeRadius || 3;
    // Edges
    for (const edge of scene.edges.values()) {
      const ap = frameSnapshot[edge.from];
      const bp = frameSnapshot[edge.to];
      if (!ap || !bp) continue;
      const sA = camera.worldToScreen(ap, cellSize, cellSize);
      const sB = camera.worldToScreen(bp, cellSize, cellSize);
      if (!sA || !sB) continue;
      ctx.strokeStyle = '#7ec8e3';
      ctx.lineWidth   = 2;
      ctx.beginPath();
      ctx.moveTo(sA.x, sA.y);
      ctx.lineTo(sB.x, sB.y);
      ctx.stroke();
    }
    // Nodes
    for (const node of scene.nodes.values()) {
      const pos = frameSnapshot[node.id];
      if (!pos) continue;
      const sp = camera.worldToScreen(pos, cellSize, cellSize);
      if (!sp) continue;
      ctx.fillStyle = '#7ec8e3';
      ctx.beginPath();
      ctx.arc(sp.x, sp.y, r, 0, Math.PI * 2);
      ctx.fill();
    }
  }
}
// =============================================================================
// Section 10: STORE (IndexedDB)
// =============================================================================
class Store {
  constructor() { this._db = null; }
  async init() {
    return new Promise((resolve, reject) => {
      const req = indexedDB.open('bonegraph', 1);
      req.onupgradeneeded = e => {
        const db = e.target.result;
        if (!db.objectStoreNames.contains('projects')) {
          db.createObjectStore('projects', { keyPath: 'id' });
        }
      };
      req.onsuccess = e => { this._db = e.target.result; resolve(); };
      req.onerror   = e => reject(e.target.error);
    });
  }
  saveProject(project)  { return this._tx('readwrite', s => s.put(project));     }
  listProjects()        { return this._tx('readonly',  s => s.getAll());          }
  loadProject(id)       { return this._tx('readonly',  s => s.get(id));           }
  deleteProject(id)     { return this._tx('readwrite', s => s.delete(id));        }
  _tx(mode, action) {
    return new Promise((resolve, reject) => {
      const tx    = this._db.transaction('projects', mode);
      const store = tx.objectStore('projects');
      const req   = action(store);
      req.onsuccess = e => resolve(e.target.result);
      req.onerror   = e => reject(e.target.error);
    });
  }
}
// =============================================================================
// Section 11: UI CONTROLLER
// =============================================================================
class UIController {
  constructor(app) {
    this.app = app;
  }
  init() {
    this._bindToolbar();
    this._bindTimeline();
    this._bindProjectButtons();
    document.getElementById('modal-close').addEventListener('click', () => this.hideModal());
    document.getElementById('modal-overlay').addEventListener('click', e => {
      if (e.target === document.getElementById('modal-overlay')) this.hideModal();
    });
  }
  // --- Toolbar ---
  _bindToolbar() {
    document.querySelectorAll('.tool-btn').forEach(btn => {
      btn.addEventListener('click', () => {
        if (btn.id === 'btn-tool-add-node') {
          this.app.addNodeAtMidpoint();
          return;
        }
        this.app.setTool(btn.dataset.tool);
      });
    });
    document.getElementById('edge-type-picker').addEventListener('change', e => {
      this.app.setEdgeType(e.target.value);
    });
    document.getElementById('canvas-size-picker').addEventListener('change', e => {
      this.app.setCanvasSize(parseInt(e.target.value, 10));
    });
    document.getElementById('btn-export').addEventListener('click', () => this.showExportModal());
  }
  setActiveTool(tool) {
    document.querySelectorAll('.tool-btn').forEach(btn => {
      btn.classList.toggle('active', btn.dataset.tool === tool);
    });
    document.getElementById('edge-type-picker').style.opacity =
      tool === TOOL.ADD_EDGE ? '1' : '0.5';
  }
  setActiveEdgeType(type) {
    document.getElementById('edge-type-picker').value = type;
  }
  // --- Timeline ---
  _bindTimeline() {
    document.getElementById('btn-add-frame').addEventListener('click', () => {
      this.app.animController.captureCurrentFrame();
      this.app.animController.addFrame();
      this.refreshFrameStrip();
    });
    document.getElementById('btn-duplicate-frame').addEventListener('click', () => {
      this.app.animController.captureCurrentFrame();
      this.app.animController.duplicateFrame(this.app.animController.currentFrameIndex);
      this.refreshFrameStrip();
    });
    document.getElementById('btn-delete-frame').addEventListener('click', () => {
      this.app.animController.removeFrame(this.app.animController.currentFrameIndex);
      this.refreshFrameStrip();
    });
    document.getElementById('btn-prev-frame').addEventListener('click', () => this.app.prevFrame());
    document.getElementById('btn-next-frame').addEventListener('click', () => this.app.nextFrame());
    document.getElementById('btn-play').addEventListener('click', () => {
      this.app.animController.isPlaying ? this.app.stopPlayback() : this.app.startPlayback();
    });
    document.getElementById('fps-input').addEventListener('change', e => {
      this.app.setFps(parseInt(e.target.value, 10));
    });
  }
  refreshFrameStrip() {
    const strip = document.getElementById('frame-strip');
    strip.innerHTML = '';
    const ac = this.app.animController;
    for (let i = 0; i < ac.frameCount; i++) {
      const thumb = document.createElement('div');
      thumb.className      = 'frame-thumb';
      thumb.dataset.index  = i;
      thumb.textContent    = i + 1;
      thumb.title          = `Frame ${i + 1}`;
      thumb.addEventListener('click', () => {
        ac.captureCurrentFrame();
        ac.setFrame(i);
        this.setActiveFrame(i);
        this.app._onFrameChange();
      });
      strip.appendChild(thumb);
    }
    this.setActiveFrame(ac.currentFrameIndex);
  }
  setActiveFrame(index) {
    document.querySelectorAll('.frame-thumb').forEach((el, i) => {
      el.classList.toggle('active', i === index);
    });
    this._updateFrameCounter();
  }
  _updateFrameCounter() {
    const el = document.getElementById('frame-counter');
    if (el) {
      const ac = this.app.animController;
      el.textContent = `Frame: ${ac.currentFrameIndex + 1} / ${ac.frameCount}`;
    }
  }
  setPlayState(playing) {
    const btn = document.getElementById('btn-play');
    if (btn) btn.textContent = playing ? '⏸' : '▶';
  }
  // --- Sidebar ---
  showNodeProperties(node) {
    document.getElementById('properties-content').innerHTML = `
      <div class="prop-group">
        <label>Name</label>
        <input type="text" id="prop-name" value="${this._esc(node.name)}">
      </div>
      <div class="prop-group">
        <label>X</label>
        <input type="number" id="prop-x" value="${node.x.toFixed(3)}" step="0.5">
      </div>
      <div class="prop-group">
        <label>Y</label>
        <input type="number" id="prop-y" value="${node.y.toFixed(3)}" step="0.5">
      </div>
      <div class="prop-group">
        <label>Z</label>
        <input type="number" id="prop-z" value="${node.z.toFixed(3)}" step="0.5">
      </div>
    `;
    document.getElementById('prop-name').addEventListener('input', e => { node.name = e.target.value; });
    for (const axis of ['x', 'y', 'z']) {
      document.getElementById(`prop-${axis}`).addEventListener('input', e => {
        const v = parseFloat(e.target.value);
        if (!isNaN(v)) { node[axis] = v; this.app.animController.captureCurrentFrame(); }
      });
    }
  }
  showEdgeProperties(edge) {
    const fn = this.app.scene.getNode(edge.from);
    const tn = this.app.scene.getNode(edge.to);
    document.getElementById('properties-content').innerHTML = `
      <div class="prop-group">
        <label>From</label><span>${this._esc(fn?.name || '?')}</span>
      </div>
      <div class="prop-group">
        <label>To</label><span>${this._esc(tn?.name || '?')}</span>
      </div>
      <div class="prop-group">
        <label>Type</label>
        <select id="prop-edge-type">
          <option value="directed"      ${edge.type === EDGE.DIRECTED      ? 'selected' : ''}>A→B Directed</option>
          <option value="bidirectional" ${edge.type === EDGE.BIDIRECTIONAL ? 'selected' : ''}>A↔B Bidirectional</option>
          <option value="elastic"       ${edge.type === EDGE.ELASTIC       ? 'selected' : ''}>A–B Elastic</option>
        </select>
      </div>
    `;
    document.getElementById('prop-edge-type').addEventListener('change', e => { edge.type = e.target.value; });
  }
  showSceneProperties() {
    const sc = this.app.scene;
    const ac = this.app.animController;
    document.getElementById('properties-content').innerHTML = `
      <div class="prop-group"><label>Nodes</label><span>${sc.nodes.size}</span></div>
      <div class="prop-group"><label>Edges</label><span>${sc.edges.size}</span></div>
      <div class="prop-group"><label>Frames</label><span>${ac ? ac.frameCount : 0}</span></div>
      <p class="hint" style="margin-top:8px">Select a node or edge</p>
    `;
  }
  clearProperties() { this.showSceneProperties(); }
  // --- Project buttons ---
  _bindProjectButtons() {
    document.getElementById('btn-new').addEventListener('click', () => {
      if (confirm('Start a new project? Unsaved changes will be lost.')) this.app.newProject();
    });
    document.getElementById('btn-save').addEventListener('click', () => this.app.saveProject());
    document.getElementById('btn-load').addEventListener('click', () => this.showProjectsModal());
  }
  // --- Modals ---
  async showProjectsModal() {
    const projects = await this.app.store.listProjects();
    let html = '<h2>Saved Projects</h2>';
    if (projects.length === 0) {
      html += '<p class="hint">No saved projects yet.</p>';
    } else {
      html += '<ul class="project-list">';
      for (const p of projects.sort((a, b) => b.updatedAt.localeCompare(a.updatedAt))) {
        html += `
          <li>
            <span>${this._esc(p.name)}</span>
            <span class="date">${new Date(p.updatedAt).toLocaleString()}</span>
            <button class="btn-load-project" data-id="${this._esc(p.id)}">Load</button>
            <button class="btn-delete-project btn-danger" data-id="${this._esc(p.id)}">Del</button>
          </li>`;
      }
      html += '</ul>';
    }
    this._showModal(html);
    document.querySelectorAll('.btn-load-project').forEach(btn => {
      btn.addEventListener('click', async () => {
        await this.app.loadProject(btn.dataset.id);
        this.hideModal();
        this.refreshFrameStrip();
      });
    });
    document.querySelectorAll('.btn-delete-project').forEach(btn => {
      btn.addEventListener('click', async () => {
        if (confirm('Delete this project?')) {
          await this.app.store.deleteProject(btn.dataset.id);
          this.showProjectsModal();
        }
      });
    });
  }
  showExportModal() {
    const getDefaultNodeDiameter = (cellSize) => {
      return Math.max(2, Math.min(8, Math.round(cellSize / 16)));
    };
    this._showModal(`
      <h2>Export Sprite Sheet</h2>
      <div class="prop-group">
        <label>Preset</label>
        <select id="export-preset">
          <option value="side_2d">2D Side Scroller</option>
          <option value="iso_4">4-way Isometric</option>
          <option value="iso_6">6-way Isometric</option>
          <option value="iso_8">8-way Isometric</option>
          <option value="custom">Custom</option>
        </select>
      </div>
      <div id="custom-angle-fields" style="display:none">
        <div class="prop-group">
          <label>Theta (radians)</label>
          <input type="number" id="export-theta" value="0" step="0.1">
        </div>
        <div class="prop-group">
          <label>Phi (radians)</label>
          <input type="number" id="export-phi" value="1.5708" step="0.1">
        </div>
      </div>
      <div class="prop-group">
        <label>Cell Size (px)</label>
        <select id="export-cell-size">
          <option value="32">32×32</option>
          <option value="64" selected>64×64</option>
          <option value="128">128×128</option>
          <option value="256">256×256</option>
        </select>
      </div>
      <div class="prop-group">
        <label>Node Diameter (px)</label>
        <input type="number" id="export-node-diameter" value="4" min="1" max="32" step="1">
      </div>
      <button id="btn-do-export" class="btn-accent">Export PNG</button>
      <div id="export-progress" style="display:none; margin-top:12px">
        <progress id="export-bar" value="0" max="1" style="width:100%"></progress>
        <span id="export-pct">0%</span>
      </div>
    `);
    const cellSizeSelect = document.getElementById('export-cell-size');
    const nodeDiameterInput = document.getElementById('export-node-diameter');
    nodeDiameterInput.value = getDefaultNodeDiameter(parseInt(cellSizeSelect.value, 10) || 64);

    document.getElementById('export-preset').addEventListener('change', e => {
      document.getElementById('custom-angle-fields').style.display =
        e.target.value === 'custom' ? '' : 'none';
    });
    cellSizeSelect.addEventListener('change', e => {
      const size = parseInt(e.target.value, 10) || 64;
      nodeDiameterInput.value = getDefaultNodeDiameter(size);
    });
    document.getElementById('btn-do-export').addEventListener('click', async () => {
      const preset   = document.getElementById('export-preset').value;
      const cellSize = parseInt(document.getElementById('export-cell-size').value, 10);
      const nodeDiameter = parseFloat(document.getElementById('export-node-diameter').value);
      const diameter = Number.isFinite(nodeDiameter) && nodeDiameter > 0 ? nodeDiameter : getDefaultNodeDiameter(cellSize);
      let customAngles = null;
      if (preset === 'custom') {
        customAngles = [{
          theta: parseFloat(document.getElementById('export-theta').value) || 0,
          phi:   parseFloat(document.getElementById('export-phi').value)   || Math.PI / 2,
          label: 'Custom'
        }];
      }
      document.getElementById('export-progress').style.display = '';
      document.getElementById('btn-do-export').disabled = true;
      await this.app.exportAnimation(preset, customAngles, cellSize, diameter, p => {
        document.getElementById('export-bar').value = p;
        document.getElementById('export-pct').textContent = Math.round(p * 100) + '%';
      });
      document.getElementById('btn-do-export').disabled = false;
    });
  }
  _showModal(html) {
    document.getElementById('modal-content').innerHTML = html;
    document.getElementById('modal-overlay').classList.remove('hidden');
  }
  hideModal() {
    document.getElementById('modal-overlay').classList.add('hidden');
  }
  setStatus(msg) {
    const el = document.getElementById('status-bar');
    if (el) el.textContent = msg;
  }
  setCanvasSize(size) {
    const el = document.getElementById('canvas-size-picker');
    if (el) el.value = size;
  }
  _esc(str) {
    return String(str)
      .replace(/&/g,  '&amp;')
      .replace(/</g,  '&lt;')
      .replace(/>/g,  '&gt;')
      .replace(/"/g,  '&quot;');
  }
}
// =============================================================================
// Section 12: APP
// =============================================================================
class App {
  constructor() {
    this._scene           = new Scene();
    this._camera          = new Camera();
    this._animController  = new AnimationController(this._scene);
    this._store           = new Store();
    this._exportMgr       = new ExportManager();
    this._renderer        = null;
    this._inputHandler    = null;
    this._ui              = null;
    this._activeTool        = TOOL.SELECT;
    this._activeEdgeType    = EDGE.DIRECTED;
    this._selectedNodeId    = null;
    this._selectedEdgeId    = null;
    this._pendingEdgeFromId = null;
    this._mouseScreenPos    = null;
    this._fps               = 12;
    this._canvasSize        = 64;
    this._projectId         = null;
    this._projectName       = 'Untitled';
  }
  async init() {
    const canvas = document.getElementById('scene-canvas');
    this._renderer     = new Renderer(canvas, this._scene, this._camera);
    this._renderer.setCanvasSize(this._canvasSize);
    this._inputHandler = new InputHandler(canvas, this);
    this._ui           = new UIController(this);
    this._ui.init();
    await this._store.init();
    this._setupResizeObserver(canvas);
    this._resizeCanvas(canvas);
    this._ui.refreshFrameStrip();
    this._ui.setActiveTool(this._activeTool);
    this._ui.showSceneProperties();
    this._renderLoop();
  }
  _setupResizeObserver(canvas) {
    const container = document.getElementById('canvas-container');
    const ro = new ResizeObserver(() => this._resizeCanvas(canvas));
    ro.observe(container);
  }
  _resizeCanvas(canvas) {
    const container = document.getElementById('canvas-container');
    canvas.width  = container.clientWidth  || 800;
    canvas.height = container.clientHeight || 600;
    this._camera.setAspect(canvas.width, canvas.height);
  }
  // --- Project ---
  newProject() {
    this._scene          = new Scene();
    this._animController = new AnimationController(this._scene);
    if (this._renderer) {
      this._renderer.scene = this._scene;
      this._renderer.setCanvasSize(this._canvasSize);
    }
    this._selectedNodeId    = null;
    this._selectedEdgeId    = null;
    this._pendingEdgeFromId = null;
    this._projectId         = null;
    this._projectName       = 'Untitled';
    this._ui.refreshFrameStrip();
    this._ui.clearProperties();
    this._ui.setStatus('New project.');
  }
  _generateId() {
    if (typeof crypto !== 'undefined' && crypto.randomUUID) return crypto.randomUUID();
    // Robust fallback: timestamp + random + monotonic counter
    App._idCounter = (App._idCounter || 0) + 1;
    return `proj-${Date.now()}-${App._idCounter}-${Math.random().toString(36).slice(2, 9)}`;
  }
  async saveProject() {
    if (!this._projectId) {
      const name = prompt('Project name:', this._projectName);
      if (name === null) return;
      this._projectName = name || 'Untitled';
      this._projectId   = this._generateId();
    }
    this._animController.captureCurrentFrame();
    const snap = this._scene.toJSON();
    const project = {
      id:         this._projectId,
      name:       this._projectName,
      updatedAt:  new Date().toISOString(),
      canvasSize: this._canvasSize,
      nodes:      snap.nodes,
      edges:      snap.edges,
      frames:     this._animController.toJSON().frames
    };
    await this._store.saveProject(project);
    this._ui.setStatus(`Saved "${this._projectName}".`);
  }
  async loadProject(id) {
    const data = await this._store.loadProject(id);
    if (!data) return;
    this._projectId    = data.id;
    this._projectName  = data.name;
    this._canvasSize   = data.canvasSize || 64;
    this._scene = new Scene();
    this._scene.fromJSON({ nodes: data.nodes || [], edges: data.edges || [] });
    this._animController = new AnimationController(this._scene);
    this._animController.fromJSON(
      { frames: data.frames || [], currentIndex: 0 },
      this._scene
    );
    if (this._renderer) {
      this._renderer.scene = this._scene;
      this._renderer.setCanvasSize(this._canvasSize);
    }
    if (this._inputHandler) this._inputHandler.app = this;
    this._selectedNodeId    = null;
    this._selectedEdgeId    = null;
    this._pendingEdgeFromId = null;
    this._ui.setCanvasSize(this._canvasSize);
    document.getElementById('canvas-size-picker').value = this._canvasSize;
    this._ui.refreshFrameStrip();
    this._ui.clearProperties();
    this._ui.setStatus(`Loaded "${this._projectName}".`);
  }
  // --- Getters ---
  get scene()             { return this._scene;           }
  get camera()            { return this._camera;          }
  get animController()    { return this._animController;  }
  get ui()                { return this._ui;              }
  get store()             { return this._store;           }
  get activeTool()        { return this._activeTool;      }
  get activeEdgeType()    { return this._activeEdgeType;  }
  get pendingEdgeFromId() { return this._pendingEdgeFromId; }
  // --- Tools ---
  setTool(tool) {
    if (this._activeTool === TOOL.ADD_EDGE) this.cancelPendingEdge();
    this._activeTool = tool;
    this._ui.setActiveTool(tool);
  }
  setEdgeType(type) {
    this._activeEdgeType = type;
    this._ui.setActiveEdgeType(type);
  }
  setCanvasSize(size) {
    this._canvasSize = size;
    if (this._renderer) this._renderer.setCanvasSize(size);
  }
  setFps(fps) {
    this._fps = Math.max(1, Math.min(60, fps));
    if (this._animController.isPlaying) { this.stopPlayback(); this.startPlayback(); }
  }
  setMousePos(pos) { this._mouseScreenPos = pos; }
  // --- Selection ---
  selectNode(id) {
    this._selectedNodeId = id;
    this._selectedEdgeId = null;
    const node = this._scene.getNode(id);
    if (node) this._ui.showNodeProperties(node);
  }
  selectEdge(id) {
    this._selectedEdgeId = id;
    this._selectedNodeId = null;
    const edge = this._scene.getEdge(id);
    if (edge) this._ui.showEdgeProperties(edge);
  }
  deselect() {
    this._selectedNodeId = null;
    this._selectedEdgeId = null;
    this._ui.showSceneProperties();
  }
  // --- Nodes / Edges ---
  addNode(x, y, z, name = null) {
    const node = this._scene.addNode(x, y, z, name);
    this._animController.captureCurrentFrame();
    this.selectNode(node.id);
    return node;
  }
  addNodeAtMidpoint(name = null) {
    const midpoint = { x: 0, y: 0, z: 0 };
    const node = this.addNode(midpoint.x, midpoint.y, midpoint.z, name);
    this._ui.setStatus('Added node at cube midpoint.');
    return node;
  }
  deleteSelected() {
    if (this._selectedNodeId !== null) {
      this._scene.removeNode(this._selectedNodeId);
      this.deselect();
      this._animController.captureCurrentFrame();
    } else if (this._selectedEdgeId !== null) {
      this._scene.removeEdge(this._selectedEdgeId);
      this.deselect();
    }
  }
  addEdge(fromId, toId, type) {
    const edge = this._scene.addEdge(fromId, toId, type);
    if (edge) this._ui.setStatus(`Edge added (${type}).`);
    else      this._ui.setStatus('Edge already exists.');
    return edge;
  }
  setPendingEdgeFrom(nodeId)  { this._pendingEdgeFromId = nodeId; }
  cancelPendingEdge()         { this._pendingEdgeFromId = null;   }
  // --- Playback ---
  startPlayback() {
    this._animController.play(this._fps, idx => {
      this._ui.setActiveFrame(idx);
      this._onFrameChange();
    });
    this._ui.setPlayState(true);
  }
  stopPlayback() {
    this._animController.stop();
    this._ui.setPlayState(false);
  }
  prevFrame() {
    this._animController.captureCurrentFrame();
    const idx = Math.max(0, this._animController.currentFrameIndex - 1);
    this._animController.setFrame(idx);
    this._ui.setActiveFrame(idx);
    this._onFrameChange();
  }
  nextFrame() {
    this._animController.captureCurrentFrame();
    const idx = Math.min(
      this._animController.frameCount - 1,
      this._animController.currentFrameIndex + 1
    );
    this._animController.setFrame(idx);
    this._ui.setActiveFrame(idx);
    this._onFrameChange();
  }
  _onFrameChange() {
    if (this._selectedNodeId !== null) {
      const node = this._scene.getNode(this._selectedNodeId);
      if (node) this._ui.showNodeProperties(node);
    }
  }
  // --- Export ---
  async exportAnimation(preset, customAngles, exportCanvasSize, nodeDiameter, onProgress) {
    this._animController.captureCurrentFrame();
    const diameter = Math.max(1, Number.isFinite(nodeDiameter) ? nodeDiameter : EXPORT_NODE_DIAMETER);
    const nodeRadius = diameter / 2;
    const dataURL = await this._exportMgr.render(
      this._scene,
      this._animController,
      preset,
      customAngles,
      exportCanvasSize,
      nodeRadius,
      onProgress
    );
    const a      = document.createElement('a');
    a.href       = dataURL;
    a.download   = `${this._projectName || 'bonegraph'}-spritesheet.png`;
    a.click();
    this._ui.setStatus('Export complete!');
  }
  // --- Render loop ---
  _renderLoop() {
    this._renderer.render(
      this._selectedNodeId,
      this._selectedEdgeId,
      this._activeTool,
      this._pendingEdgeFromId,
      this._mouseScreenPos
    );
    requestAnimationFrame(() => this._renderLoop());
  }
}
// =============================================================================
// Section 13: BOOTSTRAP
// =============================================================================
window.addEventListener('DOMContentLoaded', async () => {
  const app = new App();
  await app.init();
  window._app = app; // available for debugging
});
