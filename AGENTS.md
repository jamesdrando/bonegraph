# BoneGraph — Agent Guidance
## Project overview
BoneGraph is a browser-based 3D skeletal animation editor that exports 2D sprite sheets.
No build step, no dependencies: one HTML, one CSS, one JS file.
## Architecture
| Module | Responsibility |
|--------|---------------|
| Vec3 / Mat4 | Pure math utilities (no state) |
| Camera | Spherical-orbit 3D camera, perspective projection |
| Scene | Graph data: nodes, edges, drag-set propagation |
| Renderer | Canvas 2D painter with 3D→2D projection |
| InputHandler | Pointer/touch events → app commands |
| AnimationController | Frame storage, playback timing |
| ExportManager | Off-screen render → sprite-sheet PNG |
| Store | IndexedDB persistence |
| UIController | DOM bindings, modals, timeline strip |
| App | Orchestrator wiring all modules together |
## Key conventions
- All state lives in Scene and AnimationController. Other classes read from them.
- Camera is a pure math object with no side effects.
- UI events dispatch to App methods only — no direct scene mutations from UI.
- IndexedDB is accessed only through Store.
- No global mutable variables except the single `app` instance.
## Adding features
- New tool: add constant to TOOL, handle in InputHandler, add button in HTML/CSS.
- New edge type: add constant to EDGE, update Scene.getDragSet(), Renderer._drawEdge().
- New export preset: add to EXPORT_PRESET, update ExportManager._getPresetAngles().
## Data model (stored in IndexedDB)
```json
{
  "id": "uuid",
  "name": "my-project",
  "updatedAt": "ISO date",
  "canvasSize": 64,
  "nodes": [{"id":1,"x":0,"y":0,"z":0,"name":"root"}],
  "edges": [{"id":1,"from":1,"to":2,"type":"directed"}],
  "frames": [{"id":1,"positions":{"1":{"x":0,"y":0,"z":0}}}]
}
```
## Running
Open `index.html` in a browser. No server required for editing; IndexedDB requires a non-file:// origin for some browsers — serve with any static server.
