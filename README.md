# ChronoSRL: Temporal Geometry for Self-Supervised Reinforcement Learning

│ [Website](https://nico-bohlinger.github.io/chronosrl_website) │ [Paper]() │ [Code](https://github.com/nico-bohlinger/chronosrl) │

Project page of ChronoSRL. The final policies run in the browser, on the seven JaxGCRL tasks in a JavaScript port of brax's spring physics and on the Unitree Go2 in MuJoCo compiled to WebAssembly, next to the results of the paper.


## Viewing locally
```
python3 -m http.server 8000
```
Then open http://localhost:8000.


## Regenerating the assets
```
python tools/export_envs.py --chronosrl <chronosrl checkout>
python tools/export_policy.py <checkpoint.pkl> <args.pkl> <task> <depth> <seed>
python tools/export_go2_sim.py --chronosrl <chronosrl checkout>
python tools/export_go2.py --xml <RL-X checkout>/rl_x/environments/custom_mujoco/robot_locomotion/robots/unitree_go2/data/plane.xml --meshopt <meshoptimizer 1.3.0 package>
node tools/render_math.mjs <katex 0.18.9 package> --vendor
tools/mujoco_wasm/build.sh <mujoco checkout> <emsdk>
```
The first command writes `assets/env` and runs in the benchmark environment of the [ChronoSRL code](https://github.com/nico-bohlinger/chronosrl), the Go2 commands run in its Go2 environment.
`export_policy.py` writes one setting to `assets/policies` and records its seed in `assets/policies/manifest.json`.


## Acknowledgements
The site vendors [three.js](https://github.com/mrdoob/three.js) (MIT), [MuJoCo](https://github.com/google-deepmind/mujoco) 3.7.0 (Apache-2.0) with [libccd](https://github.com/danfis/libccd) (BSD-3-Clause), [KaTeX](https://github.com/KaTeX/KaTeX) (MIT) and the [meshoptimizer](https://github.com/zeux/meshoptimizer) decoder (MIT).
The task files in `assets/env` are converted from [JaxGCRL](https://github.com/MichalBortkiewicz/JaxGCRL) (Apache-2.0), and the Go2 model and meshes in `assets/go2` are Unitree's from [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie) (BSD-3-Clause).
Each license is next to the files it covers.
